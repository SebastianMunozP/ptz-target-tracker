package utils

import (
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

type PTZValues struct {
	Pan  float64
	Tilt float64
	Zoom float64
}

// PTZMeasurement is your calibration point structure
type PTZMeasurement struct {
	Pan            float64
	Tilt           float64
	TargetPosition r3.Vector
}

// CameraLimits defines the physical angular limits of the PTZ camera
type CameraLimits struct {
	PanMinNormalized  float64
	PanMaxNormalized  float64
	TiltMinNormalized float64
	TiltMaxNormalized float64
	ZoomMinNormalized float64
	ZoomMaxNormalized float64
	PanMinDeg         float64
	PanMaxDeg         float64
	TiltMinDeg        float64
	TiltMaxDeg        float64
	ZoomMinDistanceMM float64
	ZoomMaxDistanceMM float64
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

func ClampPTZValues(ptzValues PTZValues, limits CameraLimits) PTZValues {
	return PTZValues{
		Pan:  Clamp(ptzValues.Pan, limits.PanMinNormalized, limits.PanMaxNormalized),
		Tilt: Clamp(ptzValues.Tilt, limits.TiltMinNormalized, limits.TiltMaxNormalized),
		Zoom: Clamp(ptzValues.Zoom, limits.ZoomMinNormalized, limits.ZoomMaxNormalized),
	}
}

// Clamp clamps a value between min and max
func Clamp(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
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

func CalculateZoom(targetPos r3.Vector, cameraPos r3.Vector, cameraLimits CameraLimits) float64 {
	distance := targetPos.Distance(cameraPos)

	// Clamp distance to [minZoomDistance, maxZoomDistance]
	// Closer = zoomed out (minZoomValue), farther = zoomed in (maxZoomValue)
	if distance <= cameraLimits.ZoomMinDistanceMM {
		return cameraLimits.ZoomMinNormalized // Closest: zoomed out
	}
	if distance >= cameraLimits.ZoomMaxDistanceMM {
		return cameraLimits.ZoomMaxNormalized // Farthest: zoomed in
	}

	// Linear interpolation between minZoomDistance and maxZoomDistance
	// Normalized distance: 0 at minZoomDistance, 1 at maxZoomDistance
	// Zoom: minZoomValue at minZoomDistance (closest), maxZoomValue at maxZoomDistance (farthest)
	if (cameraLimits.ZoomMaxDistanceMM - cameraLimits.ZoomMinDistanceMM) > 0 {
		normalizedDistance := (distance - cameraLimits.ZoomMinDistanceMM) / (cameraLimits.ZoomMaxDistanceMM - cameraLimits.ZoomMinDistanceMM)
		zoom := cameraLimits.ZoomMinNormalized + (cameraLimits.ZoomMaxNormalized-cameraLimits.ZoomMinNormalized)*normalizedDistance
		return zoom
	} else {
		return cameraLimits.ZoomMinNormalized
	}
}
