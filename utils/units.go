package utils

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// PTZMeasurement is your calibration point structure
type PTZMeasurement struct {
	Pan  float64
	Tilt float64
	X    float64
	Y    float64
	Z    float64
}

// CameraLimits defines the physical angular limits of the PTZ camera
type CameraLimits struct {
	PanMinDeg  float64
	PanMaxDeg  float64
	TiltMinDeg float64
	TiltMaxDeg float64
}

var DefaultCameraLimits = CameraLimits{
	PanMinDeg:  0.0,
	PanMaxDeg:  355.0,
	TiltMinDeg: 5.0,
	TiltMaxDeg: 90.0,
}

// PanTiltResult contains computed pan/tilt values and validity
type PanTiltResult struct {
	PanNormalized  float64
	TiltNormalized float64
	PanDegrees     float64
	TiltDegrees    float64
	IsValid        bool
	ErrorMessage   string
}

// CameraPose represents the solved camera position and orientation using Viam's Pose
type CameraPose struct {
	Pose   spatialmath.Pose
	Limits CameraLimits
}

// Helper functions for coordinate conversions
func NormalizedToDegrees(normalized float64, minDeg, maxDeg float64) float64 {
	if (maxDeg - minDeg) == 0 {
		panic("maxDeg and minDeg cannot be equal")
	}
	return (normalized+1.0)*(maxDeg-minDeg)/2.0 + minDeg
}

func DegreesToNormalized(degrees, minDeg, maxDeg float64) float64 {
	if (maxDeg - minDeg) == 0 {
		panic("maxDeg and minDeg cannot be equal")
	}
	return 2.0*(degrees-minDeg)/(maxDeg-minDeg) - 1.0
}

func DegreesToRadians(degrees float64) float64 {
	return degrees * math.Pi / 180.0
}

func RadiansToDegrees(radians float64) float64 {
	return radians * 180.0 / math.Pi
}

func NormalizedToRadians(normalized float64, minDeg, maxDeg float64) float64 {
	degrees := NormalizedToDegrees(normalized, minDeg, maxDeg)
	return DegreesToRadians(degrees)
}

// PanTiltToDirection converts pan/tilt angles (in radians) to unit direction vector
// Coordinate system: +X=Right, +Y=Down, +Z=Forward (standard camera/CV convention)
// Positive pan rotates camera right (toward +X in camera frame)
// Positive tilt rotates camera up (toward -Y in camera frame, for upward-facing camera)
func PanTiltToDirection(panRad, tiltRad float64) r3.Vector {
	x := math.Cos(tiltRad) * math.Sin(panRad) // Positive because +pan = camera right = +X
	y := -math.Sin(tiltRad)                   // Negative because +tilt = up = -Y (upward-facing camera)
	z := math.Cos(tiltRad) * math.Cos(panRad) // Forward is +Z
	return r3.Vector{X: x, Y: y, Z: z}
}

// TransformPointToCameraFrame transforms a world point to camera frame
// This applies the inverse transformation: R^T * (point - t)
func TransformPointToCameraFrame(cameraPose spatialmath.Pose, worldPoint r3.Vector) r3.Vector {
	// Subtract translation: (point - t)
	translation := cameraPose.Point()
	diff := r3.Vector{
		X: worldPoint.X - translation.X,
		Y: worldPoint.Y - translation.Y,
		Z: worldPoint.Z - translation.Z,
	}

	// Apply inverse rotation: R^T * diff
	// For rotation matrices, transpose = inverse
	rotMat := cameraPose.Orientation().RotationMatrix()

	return r3.Vector{
		X: rotMat.At(0, 0)*diff.X + rotMat.At(1, 0)*diff.Y + rotMat.At(2, 0)*diff.Z,
		Y: rotMat.At(0, 1)*diff.X + rotMat.At(1, 1)*diff.Y + rotMat.At(2, 1)*diff.Z,
		Z: rotMat.At(0, 2)*diff.X + rotMat.At(1, 2)*diff.Y + rotMat.At(2, 2)*diff.Z,
	}
}
