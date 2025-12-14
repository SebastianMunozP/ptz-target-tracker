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

	camPose := TransformPointToCameraFrame(cameraPose, targetPoint)
	camLocation := camPose.Point()
	if !vectorsAlmostEqual(camLocation, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point in front: got %+v, want %+v", camLocation, expectedCamPoint)
	}

	// Test point to the right of camera
	targetPoint = r3.Vector{X: 10, Y: 0, Z: 0}
	expectedCamPoint = r3.Vector{X: 10, Y: 0, Z: 0}

	camPose = TransformPointToCameraFrame(cameraPose, targetPoint)
	camLocation = camPose.Point()
	if !vectorsAlmostEqual(camLocation, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point to the right: got %+v, want %+v", camLocation, expectedCamPoint)
	}

	// Test point above the camera
	targetPoint = r3.Vector{X: 0, Y: 10, Z: 0}
	expectedCamPoint = r3.Vector{X: 0, Y: 10, Z: 0}

	camPose = TransformPointToCameraFrame(cameraPose, targetPoint)
	camLocation = camPose.Point()
	if !vectorsAlmostEqual(camLocation, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point above: got %+v, want %+v", camLocation, expectedCamPoint)
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
	targetPoint := r3.Vector{X: 0, Y: 0, Z: 0}
	expectedCamPoint := r3.Vector{X: -1000, Y: -1000, Z: 0}

	camPose := TransformPointToCameraFrame(cameraPose, targetPoint)
	camLocation := camPose.Point()
	camOrientation := camPose.Orientation().OrientationVectorDegrees()
	if !vectorsAlmostEqual(camLocation, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point in front: got %+v, want %+v", camLocation, expectedCamPoint)
	}
	if orientation.Theta != camOrientation.Theta || orientation.OX != camOrientation.OX ||
		orientation.OY != camOrientation.OY || orientation.OZ != camOrientation.OZ {
		t.Errorf("Camera orientation changed unexpectedly: got %+v, want %+v", orientation, camOrientation)
	}

	// Test point located above the camera in world coordinates
	// At viam +Z is up
	targetPoint = r3.Vector{X: 1000, Y: 1000, Z: 1000}
	// For the camera, -Y is up
	expectedCamPoint = r3.Vector{X: 0, Y: 0, Z: 1000}

	camPose = TransformPointToCameraFrame(cameraPose, targetPoint)
	camLocation = camPose.Point()
	if !vectorsAlmostEqual(camLocation, expectedCamPoint, 1e-6) {
		t.Errorf("Transform to camera frame failed for point above: got %+v, want %+v", camLocation, expectedCamPoint)
	}
	if orientation.Theta != camOrientation.Theta || orientation.OX != camOrientation.OX ||
		orientation.OY != camOrientation.OY || orientation.OZ != camOrientation.OZ {
		t.Errorf("Camera orientation changed unexpectedly: got %+v, want %+v", orientation, camOrientation)
	}
}
