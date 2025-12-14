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

// Helper to convert spatialmath.Pose to a user-friendly map
func PoseToMap(pose spatialmath.Pose) map[string]interface{} {
	if pose == nil {
		return nil
	}
	pos := pose.Point()
	ori := pose.Orientation().Quaternion()
	return map[string]interface{}{
		"translation": map[string]float64{
			"x": pos.X,
			"y": pos.Y,
			"z": pos.Z,
		},
		"orientation": map[string]float64{
			"Imag": ori.Imag,
			"Jmag": ori.Jmag,
			"Kmag": ori.Kmag,
			"Real": ori.Real,
		},
	}
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
	return math.Max(min, math.Min(max, value))
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

func CalculatePanTiltInCameraFrame(targetPositionInCameraFrame r3.Vector) (panRad, tiltRad float64) {
	x, y, z := targetPositionInCameraFrame.X, targetPositionInCameraFrame.Y, targetPositionInCameraFrame.Z

	// Pan: atan2(x, z)
	panRad = math.Atan2(x, z)

	// Tilt: atan2(-y, sqrt(x^2 + z^2))
	rXZ := math.Sqrt(x*x + z*z)
	tiltRad = math.Atan2(-y, rXZ) // Negative because +tilt = up = -Y (upward-facing camera)

	return panRad, tiltRad
}

// TransformPointToCameraFrame using Viam's Pose API
func TransformPointToCameraFrame(cameraPose spatialmath.Pose, worldPoint r3.Vector) spatialmath.Pose {
	// Create a "pose" for the point (position only, no rotation)
	pointPose := spatialmath.NewPose(worldPoint, &spatialmath.OrientationVector{
		Theta: 0, OX: 0, OY: 0, OZ: 1,
	})

	// Compose: camera_inverse * point transforms point to camera frame
	// This uses Viam's internal pose math
	cameraPoseInverse := spatialmath.PoseInverse(cameraPose)
	resultPose := spatialmath.Compose(cameraPoseInverse, pointPose)

	return resultPose
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
