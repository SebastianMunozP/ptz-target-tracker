package trackers

import (
	"math"
	"ptztargettracker/utils"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func TestEndToEnd(t *testing.T) {
	// For this test, we'll assume the camera is at position (-1, -1, 0)
	// Viam world frame: +X=Right, +Y=Forward, +Z=Up
	// Camera at ground level (world Z=0), positioned at (-1, -1) in XY plane
	// Camera uses internal convention: +X=Right, +Y=Down, +Z=Forward
	// We'll create synthetic measurements by:
	// 1. Picking target points (above camera in world frame, Z > 0)
	// 2. Computing what pan/tilt would see those points from camera position
	// 3. Using those as our "measurements"
	// Then we solve to recover the camera position

	trueCameraPos := r3.Vector{X: -1.0, Y: -1.0, Z: 0.0}
	// Camera looking horizontally forward: rotate 90° around +X axis
	// This makes camera's +Z (forward) point along world +Y (forward)
	// Camera's +Y (down) points along world +X (right)
	// Camera's +X (right) points along world -Y (backward)
	trueCameraOri := &spatialmath.OrientationVector{
		Theta: math.Pi / 2, // 90 degrees
		OX:    1,           // Rotate around X axis
		OY:    0,
		OZ:    0,
	}
	truePose := spatialmath.NewPose(trueCameraPos, trueCameraOri)

	// Target points in Viam world frame (+X=Right, +Y=Forward, +Z=Up)
	// All above camera world Z=0, spread out in XY plane
	targetPoints := []r3.Vector{
		{X: 0.0, Y: 1.0, Z: 2.0},  // Right/forward of camera, above
		{X: -2.0, Y: 1.0, Z: 2.0}, // Left of camera, forward, above
		{X: -0.5, Y: 0.5, Z: 2.5}, // Slightly right of camera, above
		{X: -1.5, Y: 0.5, Z: 2.5}, // Slightly left of camera, above
		{X: -1.0, Y: 0.0, Z: 1.0}, // Directly in front of camera, above
		{X: -0.7, Y: 2.0, Z: 1.5}, // Right/far forward, above
		{X: -1.3, Y: 2.0, Z: 1.5}, // Left/far forward, above
		{X: -0.2, Y: 1.0, Z: 1.8}, // Right/forward, above
	}

	var DefaultCameraLimits = utils.CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  355.0,
		TiltMinDeg: 5.0,
		TiltMaxDeg: 90.0,
	}

	limits := DefaultCameraLimits

	// Generate synthetic measurements
	measurements := make([]utils.PTZMeasurement, len(targetPoints))
	for i, target := range targetPoints {
		// Transform target to camera frame
		camPoint := utils.TransformPointToCameraFrame(truePose, target)
		// Compute pan/tilt in radians
		// Camera uses Y-down convention: +X=Right, +Y=Down, +Z=Forward
		// Positive pan = camera rotates right (toward +X)
		panRad := math.Atan2(camPoint.X, camPoint.Z)
		rXZ := math.Sqrt(camPoint.X*camPoint.X + camPoint.Z*camPoint.Z)
		// Positive tilt = looking up = -Y in camera frame (upward-facing camera)
		tiltRad := math.Atan2(-camPoint.Y, rXZ)

		// Convert to degrees
		panDeg := utils.RadiansToDegrees(panRad)
		tiltDeg := utils.RadiansToDegrees(tiltRad)

		// Convert to normalized
		panNorm := utils.DegreesToNormalized(panDeg, limits.PanMinDeg, limits.PanMaxDeg)
		tiltNorm := utils.DegreesToNormalized(tiltDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

		measurements[i] = utils.PTZMeasurement{
			Pan:  panNorm,
			Tilt: tiltNorm,
			TargetPosition: r3.Vector{
				X: target.X,
				Y: target.Y,
				Z: target.Z,
			},
		}
	}

	t.Logf("PTZ Camera Calibration")
	t.Logf("======================")
	t.Logf("Camera: Pan=[%.0f°,%.0f°], Tilt=[%.0f°,%.0f°]\n\n",
		limits.PanMinDeg, limits.PanMaxDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

	utils.ValidateMeasurements(measurements)

	t.Logf("\nSolving for camera pose...")
	cameraPose, err := SolveCameraPose(measurements, limits)
	if err != nil {
		t.Errorf("Error: %v", err)
		return
	}

	// Print pose information
	point := cameraPose.Pose.Point()
	orientation := cameraPose.Pose.Orientation()

	// Try to get axis-angle representation
	ov := orientation.AxisAngles()
	t.Logf("\nCamera Pose:  Position: [%.3f, %.3f, %.3f], Orientation: [theta: %.3f rad (%.1f°), axis: [%.3f, %.3f, %.3f]]\n",
		point.X, point.Y, point.Z, ov.Theta, ov.Theta*180/math.Pi, ov.RX, ov.RY, ov.RZ)

	// You can also convert to other representations
	quat := orientation.Quaternion()
	t.Logf("  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]\n",
		quat.Real, quat.Imag, quat.Jmag, quat.Kmag)

	utils.ValidateCalibration(measurements, cameraPose, XYZToPanTilt)

	// Example usage
	t.Logf("\n\nExample: Track new target")
	result := XYZToPanTilt(r3.Vector{X: 0.8, Y: 0.3, Z: 2.2}, cameraPose)
	t.Logf("Target: [0.800, 0.300, 2.200]\n")
	if result.IsValid {
		t.Logf("✓ Pan: %.3f, Tilt: %.3f (normalized)\n",
			result.PanNormalized, result.TiltNormalized)
		t.Logf("  (%.1f°, %.1f°)\n", result.PanDegrees, result.TiltDegrees)
	} else {
		t.Logf("✗ %s\n", result.ErrorMessage)
	}
}
