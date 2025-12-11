package ptzcalibrators

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func vectorsAlmostEqual(v1, v2 r3.Vector, tol float64) bool {
	return abs(v1.X-v2.X) < tol && abs(v1.Y-v2.Y) < tol && abs(v1.Z-v2.Z) < tol
}

func abs(a float64) float64 {
	if a < 0 {
		return -a
	}
	return a
}

func TestNormalizedToDegreesAndBack(t *testing.T) {
	// Test values
	minDeg := -90.0
	maxDeg := 90.0

	testValues := []float64{-1.0, -0.5, 0.0, 0.5, 1.0}
	expectedDegrees := []float64{-90.0, -45.0, 0.0, 45.0, 90.0}

	for i, norm := range testValues {
		deg := NormalizedToDegrees(norm, minDeg, maxDeg)
		expectedDeg := expectedDegrees[i]
		if deg != expectedDeg {
			t.Errorf("Normalized to degrees failed: got %f, want %f", deg, expectedDeg)
		}
		normBack := DegreesToNormalized(deg, minDeg, maxDeg)

		if norm != normBack {
			t.Errorf("Normalized to radians and back failed: got %f, want %f", normBack, norm)
		}
	}
}

func TestNormalizedToDegreesAndBack2(t *testing.T) {
	// Test values
	minDeg := 5.0
	maxDeg := 90.0

	testValues := []float64{-1.0, -0.5, 0.0, 0.5, 1.0}
	expectedDegrees := []float64{5.0, 26.25, 47.5, 68.75, 90.0}

	for i, norm := range testValues {
		deg := NormalizedToDegrees(norm, minDeg, maxDeg)
		expectedDeg := expectedDegrees[i]
		if deg != expectedDeg {
			t.Errorf("Normalized to degrees failed: got %f, want %f", deg, expectedDeg)
		}
		normBack := DegreesToNormalized(deg, minDeg, maxDeg)

		if norm != normBack {
			t.Errorf("Normalized to radians and back failed: got %f, want %f", normBack, norm)
		}
	}
}

func TestDegreesToRadiansAndBack(t *testing.T) {
	// Test values
	testValues := []float64{-180.0, -90.0, 0.0, 90.0, 180.0}
	expectedRadians := []float64{-3.141592653589793, -1.5707963267948966, 0.0, 1.5707963267948966, 3.141592653589793}

	for i, deg := range testValues {
		rad := DegreesToRadians(deg)
		expectedRad := expectedRadians[i]
		if rad != expectedRad {
			t.Errorf("Degrees to radians failed: got %f, want %f", rad, expectedRad)
		}
		degBack := RadiansToDegrees(rad)

		if deg != degBack {
			t.Errorf("Radians to degrees and back failed: got %f, want %f", degBack, deg)
		}
	}
}

func TestPanTiltToDirection(t *testing.T) {
	// Test values
	testValues := []struct {
		panDeg  float64
		tiltDeg float64
	}{
		{0.0, 0.0},
		{90.0, 0.0},
		{0.0, 90.0},
		{-90.0, 0.0},
		{0.0, -90.0},
	}

	/*
		Camera coordinate system (standard CV convention): +X=Right, +Y=Down, +Z=Forward
		Positive pan rotates camera right (into +X direction in camera frame).
		Positive tilt rotates camera up (into -Y direction, for upward-facing camera).
		Expected direction vectors:
		- Pan = 0, Tilt = 0 -> Direction = (0, 0, 1)  - straight ahead
		- Pan = 90, Tilt = 0 -> Direction = (1, 0, 0) - right
		- Pan = 0, Tilt = 90 -> Direction = (0, -1, 0) - up
		- Pan = -90, Tilt = 0 -> Direction = (-1, 0, 0) - left
		- Pan = 0, Tilt = -90 -> Direction = (0, 1, 0) - down
	*/
	expectedDirections := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: -1, Z: 0},
		{X: -1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
	}

	for i, tv := range testValues {
		dir := PanTiltToDirection(DegreesToRadians(tv.panDeg), DegreesToRadians(tv.tiltDeg))
		expectedDir := expectedDirections[i]
		if !vectorsAlmostEqual(dir, expectedDir, 1e-6) {
			t.Errorf("Pan and tilt to direction failed in test case %d: got %+v, want %+v", i, dir, expectedDir)
		}
	}
}

func TestNormalizedPanTiltToDirection(t *testing.T) {
	// Test values
	minPanDeg := 0.0
	maxPanDeg := 355.0
	minTiltDeg := 5.0
	maxTiltDeg := 90.0

	testValues := []struct {
		normPan  float64
		normTilt float64
	}{
		{-1.0, -1.0},
		{-0.5, -0.5},
		{0.0, 0.0},
		{0.5, 0.5},
		{1.0, 1.0},
	}

	// Expected pan/tilt in degrees
	expectedPanTiltDeg := []struct {
		panDeg  float64
		tiltDeg float64
	}{
		{0.0, 5.0},
		{88.75, 26.25},
		{177.5, 47.5},
		{266.25, 68.75},
		{355.0, 90.0},
	}

	for i, tv := range testValues {
		panDeg := NormalizedToDegrees(tv.normPan, minPanDeg, maxPanDeg)
		tiltDeg := NormalizedToDegrees(tv.normTilt, minTiltDeg, maxTiltDeg)

		expectedPan := expectedPanTiltDeg[i].panDeg
		expectedTilt := expectedPanTiltDeg[i].tiltDeg

		if panDeg != expectedPan || tiltDeg != expectedTilt {
			t.Errorf("Normalized to degrees failed in test case %d: got pan %f, tilt %f; want pan %f, tilt %f", i, panDeg, tiltDeg, expectedPan, expectedTilt)
		}

		dir := PanTiltToDirection(DegreesToRadians(panDeg), DegreesToRadians(tiltDeg))
		// Just print the direction for visual inspection
		t.Logf("Test case %d: Normalized Pan: %f, Normalized Tilt: %f => Direction Vector: %+v", i, tv.normPan, tv.normTilt, dir)
	}
}

func TestTransformPointToCameraFrame(t *testing.T) {
	// Define a camera pose at origin, looking along +Z, with no rotation
	traslation := r3.Vector{X: 0, Y: 0, Z: 0}
	orientation := &spatialmath.OrientationVector{
		Theta: 0,
		OX:    0,
		OY:    0,
		OZ:    1,
	}
	cameraPose := spatialmath.NewPose(traslation, orientation)

	// Test point directly in front of camera
	targetPoint := r3.Vector{X: 0, Y: 0, Z: 10}
	expectedCamPoint := r3.Vector{X: 0, Y: 0, Z: 10}

	camPoint := TransformPointToCameraFrame(cameraPose, targetPoint)
	if !vectorsAlmostEqual(camPoint, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point in front: got %+v, want %+v", camPoint, expectedCamPoint)
	}

	// Test point to the right of camera
	targetPoint = r3.Vector{X: 10, Y: 0, Z: 0}
	expectedCamPoint = r3.Vector{X: 10, Y: 0, Z: 0}

	camPoint = TransformPointToCameraFrame(cameraPose, targetPoint)
	if !vectorsAlmostEqual(camPoint, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point to the right: got %+v, want %+v", camPoint, expectedCamPoint)
	}

	// Test point above the camera
	targetPoint = r3.Vector{X: 0, Y: 10, Z: 0}
	expectedCamPoint = r3.Vector{X: 0, Y: 10, Z: 0}

	camPoint = TransformPointToCameraFrame(cameraPose, targetPoint)
	if !vectorsAlmostEqual(camPoint, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point above: got %+v, want %+v", camPoint, expectedCamPoint)
	}
}

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

	var DefaultCameraLimits = CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  355.0,
		TiltMinDeg: 5.0,
		TiltMaxDeg: 90.0,
	}

	limits := DefaultCameraLimits

	// Generate synthetic measurements
	measurements := make([]PTZMeasurement, len(targetPoints))
	for i, target := range targetPoints {
		// Transform target to camera frame
		camPoint := TransformPointToCameraFrame(truePose, target)

		// Compute pan/tilt in radians
		// Camera uses Y-down convention: +X=Right, +Y=Down, +Z=Forward
		// Positive pan = camera rotates right (toward +X)
		panRad := math.Atan2(camPoint.X, camPoint.Z)
		rXZ := math.Sqrt(camPoint.X*camPoint.X + camPoint.Z*camPoint.Z)
		// Positive tilt = looking up = -Y in camera frame (upward-facing camera)
		tiltRad := math.Atan2(-camPoint.Y, rXZ)

		// Convert to degrees
		panDeg := RadiansToDegrees(panRad)
		tiltDeg := RadiansToDegrees(tiltRad)

		// Convert to normalized
		panNorm := DegreesToNormalized(panDeg, limits.PanMinDeg, limits.PanMaxDeg)
		tiltNorm := DegreesToNormalized(tiltDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

		measurements[i] = PTZMeasurement{
			Pan:  panNorm,
			Tilt: tiltNorm,
			X:    target.X,
			Y:    target.Y,
			Z:    target.Z,
		}
	}

	t.Logf("PTZ Camera Calibration")
	t.Logf("======================")
	t.Logf("Camera: Pan=[%.0f°,%.0f°], Tilt=[%.0f°,%.0f°]\n\n",
		limits.PanMinDeg, limits.PanMaxDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

	ValidateCalibration(measurements)

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

	TestCalibration(measurements, cameraPose)

	// Example usage
	t.Logf("\n\nExample: Track new target")
	result := XYZToPanTilt(0.8, 0.3, 2.2, cameraPose)
	t.Logf("Target: [0.800, 0.300, 2.200]\n")
	if result.IsValid {
		t.Logf("✓ Pan: %.3f, Tilt: %.3f (normalized)\n",
			result.PanNormalized, result.TiltNormalized)
		t.Logf("  (%.1f°, %.1f°)\n", result.PanDegrees, result.TiltDegrees)
	} else {
		t.Logf("✗ %s\n", result.ErrorMessage)
	}
}
