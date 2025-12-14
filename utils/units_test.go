package utils

import (
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

func TestTransformPointToCameraFrameCameraNotAtOrigin(t *testing.T) {
	// Define a camera pose at (1000, 1000, 0), looking along +Z, with no rotation
	// The location of the camera is in world coordinates
	traslation := r3.Vector{X: 1000, Y: 1000, Z: 0}
	orientation := &spatialmath.OrientationVector{
		Theta: 0,
		OX:    0,
		OY:    0,
		OZ:    1,
	}
	cameraPose := spatialmath.NewPose(traslation, orientation)

	// Test point located at origin in world coordinates
	targetPoint := r3.Vector{X: 0, Y: 0, Z: 00}
	expectedCamPoint := r3.Vector{X: -1000, Y: -1000, Z: 0}

	camPoint := TransformPointToCameraFrame(cameraPose, targetPoint)
	if !vectorsAlmostEqual(camPoint, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point in front: got %+v, want %+v", camPoint, expectedCamPoint)
	}

	// Test point located above the camera in world coordinates
	// At viam +Z is up
	targetPoint = r3.Vector{X: 1000, Y: 1000, Z: 1000}
	// For the camera, -Y is up
	expectedCamPoint = r3.Vector{X: 0, Y: -1000, Z: 0}
}
