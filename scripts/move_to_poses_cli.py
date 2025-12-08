#!/usr/bin/env python3
"""
Script to move arm through calibration poses.
Sets arm speed at the beginning, uses Viam CLI for moves, restores speed at the end.

Configuration:
    Create a .env file in the project root with:
        VIAM_API_KEY=your-api-key
        VIAM_API_KEY_ID=your-api-key-id
        VIAM_ROBOT_ADDRESS=your-robot-address
        VIAM_COMPONENT_NAME=your-component-name
        VIAM_PART_ID=your-part-id
    
Usage: 
    python move_to_poses_cli.py [poses_file] [--speed-percent 25]
"""

import json
import sys
import argparse
import os
import asyncio
import subprocess
from pathlib import Path
from dotenv import load_dotenv

from viam.robot.client import RobotClient
from viam.components.arm import Arm

        
        
DEFAULT_VELOCITY_NORMAL = 10
DEFAULT_VELOCITY_SLOW = 10

async def connect_robot(api_key: str, api_key_id: str, robot_address: str):
    """Connect to the robot."""
    opts = RobotClient.Options.with_api_key(
        api_key=api_key,
        api_key_id=api_key_id
    )
    return await RobotClient.at_address(robot_address, opts)


async def set_arm_speed(arm: Arm, speed_percent: float):
    """Set arm speed via DoCommand."""
    try:
        await arm.do_command({"set_vel": speed_percent})
        return True
    except Exception as e:
        print(f"Error setting arm speed: {e}")
        return False


async def get_arm_speed(arm: Arm):
    """Get current arm speed setting."""
    try:
        result = await arm.do_command({"get_speed_pct": True})
        if "speed_pct" in result:
            return result["speed_pct"]
    except Exception:
        pass
    return None


def move_to_pose(pose_entry, part_id: str, component: str):
    """
    Move to a specific pose using Viam CLI.
    
    Args:
        pose_entry: Pose data dictionary
        part_id: Viam part ID
        component: Component name
        
    Returns:
        tuple: (success: bool, stdout: str, stderr: str)
    """
    try:
        pose = pose_entry['data']['pose']
        x, y, z = pose['x'], pose['y'], pose['z']
        ox, oy, oz = pose['o_x'], pose['o_y'], pose['o_z']
        theta = pose['theta']
    except KeyError as e:
        return False, "", f"Missing field in pose: {e}"
    
    # Build CLI command
    cmd = [
        'sudo', 'viam', 'machines', 'part', 'motion', 'set-pose',
        '--part', part_id,
        '--component', component,
        '-x', str(x), '-y', str(y), '-z', str(z),
        '--ox', str(ox), '--oy', str(oy), '--oz', str(oz),
        '--theta', str(theta)
    ]
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        return True, result.stdout, ""
    except subprocess.CalledProcessError as e:
        return False, "", str(e.stderr)


async def main_async():
    parser = argparse.ArgumentParser(
        description='Move arm through calibration poses with speed control',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--poses-file',
        nargs='?',
        default='ee_poses_generated.json',
        help='JSON file containing poses (default: ee_poses_generated.json)'
    )
    
    parser.add_argument(
        '--env-file',
        required=True,
        help='Path to .env file with robot configuration'
    )
    
    args = parser.parse_args()
    
    # Load .env file (required)
    env_path = Path(args.env_file)
    if not env_path.exists():
        print(f"Error: .env file not found: {args.env_file}")
        sys.exit(1)
    
    load_dotenv(env_path)
    print(f"Loaded configuration from: {args.env_file}")
    print()
    
    # Get values with priority: CLI args > .env > defaults
    api_key = os.getenv('VIAM_API_KEY')
    api_key_id = os.getenv('VIAM_API_KEY_ID')
    robot_address = os.getenv('VIAM_ROBOT_ADDRESS')
    component = os.getenv('VIAM_COMPONENT_NAME', 'ptz_fake_arm_2')
    part_id = os.getenv('VIAM_PART_ID')
    
    # Validate
    if not all([api_key, api_key_id, robot_address, part_id]):
        print("Error: Missing required configuration!")
        print("Set VIAM_API_KEY, VIAM_API_KEY_ID, VIAM_ROBOT_ADDRESS, VIAM_PART_ID")
        print("Either in .env file or via command line arguments")
        sys.exit(1)
    
    # Load poses
    poses_path = Path(args.poses_file)
    if not poses_path.exists():
        print(f"Error: Poses file '{args.poses_file}' not found!")
        sys.exit(1)
    
    with open(poses_path, 'r') as f:
        poses = json.load(f)
    
    total_poses = len(poses)
    print(f"Found {total_poses} poses in {args.poses_file}")
    print(f"Component: {component}")
    print(f"Target speed: {DEFAULT_VELOCITY_NORMAL}%")
    print()
    
    # Connect to robot
    print("Connecting to robot...")
    robot = await connect_robot(api_key, api_key_id, robot_address)
    arm = Arm.from_robot(robot, component)
    print("✓ Connected")
    
    # Get original speed
    original_speed = await get_arm_speed(arm)
    if original_speed:
        print(f"Current arm speed: {original_speed}%")
    
    # Set slow speed
    print(f"Setting arm speed to {DEFAULT_VELOCITY_NORMAL}%...")
    if await set_arm_speed(arm, DEFAULT_VELOCITY_NORMAL):
        print("✓ Speed set")
    else:
        print("⚠ Arm does not support speed control via SDK")
        print("  You may need to configure speed limits in your robot config")
        print("  or manually set speed on the arm controller")
        print()
        response = input("Continue anyway? [y/N]: ")
        if response.lower() != 'y':
            print("Aborted.")
            await robot.close()
            sys.exit(0)
    
    # Close connection (CLI will handle moves)
    await robot.close()
    print()
    
    input("Press Enter to start moving through poses...")
    print()
    
    # Track results
    visited_poses = []  # List of (index, pose_entry) tuples
    failed_poses = []   # List of (index, pose_entry, error) tuples
    
    # Move through poses using CLI with ability to go back
    i = 0
    while i < total_poses:
        pose_entry = poses[i]
        
        try:
            pose = pose_entry['data']['pose']
            x, y, z = pose['x'], pose['y'], pose['z']
            ox, oy, oz = pose['o_x'], pose['o_y'], pose['o_z']
            theta = pose['theta']
        except KeyError as e:
            print(f"Error: Missing field in pose {i + 1}: {e}")
            i += 1
            continue
        
        print("=" * 50)
        print(f"Pose {i + 1} of {total_poses}")
        print("=" * 50)
        print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"Orientation: ({ox:.4f}, {oy:.4f}, {oz:.4f}) @ {theta:.2f}°")
        print()
        
        print("Moving to pose...")
        success, stdout, stderr = move_to_pose(pose_entry, part_id, component)
        
        if success:
            print("✓ Move completed")
            # Update visited list - remove if already there, add at current position
            visited_poses = [(idx, p) for idx, p in visited_poses if idx != i]
            visited_poses.append((i, pose_entry))
            if stdout.strip():
                print(stdout)
        else:
            print(f"✗ Move failed: {stderr}")
            # Update failed list - remove if already there, add at current position
            failed_poses = [(idx, p, e) for idx, p, e in failed_poses if idx != i]
            failed_poses.append((i, pose_entry, stderr))
        
        if i < total_poses - 1 or i > 0:
            print()
            try:
                user_input = input("Press Enter for next pose, 'p' for previous pose, or Ctrl+C to stop: ").strip().lower()
                if user_input == 'p' and i > 0:
                    i -= 1
                    print(f"\n↩ Going back to pose {i + 1}")
                    print()
                else:
                    i += 1
                    print()
            except KeyboardInterrupt:
                print("\n\nStopped by user.")
                break
        else:
            i += 1
    
    # Restore original speed
    if original_speed:
        print()
        print("Restoring original arm speed...")
        robot = await connect_robot(api_key, api_key_id, robot_address)
        arm = Arm.from_robot(robot, component)
        await set_arm_speed(arm, original_speed)
        print(f"✓ Speed restored to {original_speed}%")
        await robot.close()
    
    print()
    print("=" * 50)
    print("All poses completed!")
    print("=" * 50)
    print()
    
    # Print summary
    print("=" * 50)
    print("CALIBRATION SUMMARY")
    print("=" * 50)
    print(f"Total poses: {total_poses}")
    print(f"✓ Successful: {len(visited_poses)} ({len(visited_poses)*100.0/total_poses:.1f}%)")
    print(f"✗ Failed: {len(failed_poses)} ({len(failed_poses)*100.0/total_poses:.1f}%)")
    print()
    
    if visited_poses:
        print(f"Visited poses ({len(visited_poses)}):")
        for idx, pose_entry in visited_poses:
            pose = pose_entry['data']['pose']
            print(f"  ✓ Pose {idx + 1}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})")
        print()
    
    if failed_poses:
        print(f"Failed poses ({len(failed_poses)}):")
        for idx, pose_entry, error in failed_poses:
            pose = pose_entry['data']['pose']
            print(f"  ✗ Pose {idx + 1}: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})")
            # Extract the key error message
            if "physically unreachable" in error:
                print(f"     Reason: Position unreachable (IK solver failed)")
            else:
                print(f"     Reason: {error.split('desc = ')[-1][:80] if 'desc = ' in error else error[:80]}")
        print()
    
    # Save results to JSON for visualization
    results_file = args.poses_file.replace('.json', '_results.json')
    results_data = {
        'total': total_poses,
        'visited': [{'index': idx, 'pose': pose_entry} for idx, pose_entry in visited_poses],
        'failed': [{'index': idx, 'pose': pose_entry, 'error': error} for idx, pose_entry, error in failed_poses]
    }
    with open(results_file, 'w') as f:
        json.dump(results_data, f, indent=2)
    print(f"Results saved to: {results_file}")
    print("=" * 50)


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
