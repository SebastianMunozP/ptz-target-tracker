package trackers

import (
	"fmt"
	"math"
	"ptztargettracker/utils"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/optimize"
)

// ResidualFunction for optimization
type ResidualFunction struct {
	Measurements []utils.PTZMeasurement
	Directions   []r3.Vector
}

func (rf *ResidualFunction) Func(params []float64) float64 {
	residuals := rf.Residuals(params)
	sum := 0.0
	for _, r := range residuals {
		sum += r * r
	}
	return sum
}

func (rf *ResidualFunction) Residuals(params []float64) []float64 {
	// params: [tx, ty, tz, rx, ry, rz] (axis-angle via OrientationVector)
	translation := r3.Vector{X: params[0], Y: params[1], Z: params[2]}

	// OrientationVector is Viam's axis-angle representation
	orientation := &spatialmath.OrientationVector{
		Theta: math.Sqrt(params[3]*params[3] + params[4]*params[4] + params[5]*params[5]),
		OX:    params[3],
		OY:    params[4],
		OZ:    params[5],
	}

	// Create pose from translation and rotation
	pose := spatialmath.NewPose(translation, orientation)

	residuals := make([]float64, len(rf.Measurements))

	for i, meas := range rf.Measurements {
		// World point
		worldPoint := r3.Vector{X: meas.TargetPosition.X, Y: meas.TargetPosition.Y, Z: meas.TargetPosition.Z}

		// Transform world point to camera frame
		camPoint := utils.TransformPointToCameraFrame(pose, worldPoint)

		// Normalize both vectors
		camNorm := camPoint.Norm()
		if camNorm < 1e-10 {
			residuals[i] = 1e10 // Large penalty if point is at camera
			continue
		}
		camDir := camPoint.Mul(1.0 / camNorm)

		// Use 1 - dot product as residual (0 when aligned, 2 when opposite)
		// This measures angular error
		dot := camDir.Dot(rf.Directions[i])
		residuals[i] = 1.0 - dot
	}

	return residuals
}

// SolveCameraPose solves for camera pose from PTZ measurements
func SolveCameraPose(measurements []utils.PTZMeasurement, limits utils.CameraLimits) (*utils.CameraPose, error) {
	// Convert normalized pan/tilt to direction vectors
	directions := make([]r3.Vector, len(measurements))
	for i, meas := range measurements {
		panRad := utils.NormalizedToRadians(meas.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		tiltRad := utils.NormalizedToRadians(meas.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)
		directions[i] = utils.PanTiltToDirection(panRad, tiltRad)
	}

	rf := &ResidualFunction{
		Measurements: measurements,
		Directions:   directions,
	}

	// Compute better initial guess: centroid of measurements
	var sumX, sumY, sumZ float64
	for _, meas := range measurements {
		sumX += meas.TargetPosition.X
		sumY += meas.TargetPosition.Y
		sumZ += meas.TargetPosition.Z
	}
	n := float64(len(measurements))
	centroidX := sumX / n
	centroidY := sumY / n
	centroidZ := sumZ / n

	// Start with camera at negative centroid (opposite side)
	// and no rotation (looking along +Z)
	x0 := []float64{-centroidX, -centroidY, -centroidZ, 0, 0, 0}

	problem := optimize.Problem{
		Func: rf.Func,
	}

	settings := &optimize.Settings{
		FuncEvaluations: 100000, // Increase max evaluations further
		Converger: &optimize.FunctionConverge{
			Absolute: 1e-6, // Further relax convergence criteria
			Relative: 1e-6,
		},
	}

	result, err := optimize.Minimize(problem, x0, settings, &optimize.NelderMead{})
	if err != nil {
		return nil, fmt.Errorf("optimization failed: %w", err)
	}

	// Accept result if residual is small enough, even if didn't formally converge
	if err := result.Status.Err(); err != nil {
		// Check if residual is acceptable (< 0.5 means avg angle error is small)
		if result.F > 0.5 {
			return nil, fmt.Errorf("optimization did not converge and residual too large (%.6f): %w", result.F, err)
		}
	}

	// Create Viam Pose from result
	translation := r3.Vector{
		X: result.X[0],
		Y: result.X[1],
		Z: result.X[2],
	}

	// OrientationVector stores axis-angle
	theta := math.Sqrt(result.X[3]*result.X[3] + result.X[4]*result.X[4] + result.X[5]*result.X[5])
	orientation := &spatialmath.OrientationVector{
		Theta: theta,
		OX:    result.X[3],
		OY:    result.X[4],
		OZ:    result.X[5],
	}

	pose := spatialmath.NewPose(translation, orientation)

	return &utils.CameraPose{
		Pose:   pose,
		Limits: limits,
	}, nil
}

// XYZToPanTilt converts world coordinates to normalized pan/tilt
func XYZToPanTilt(targetPosition r3.Vector, cameraPose *utils.CameraPose) utils.PanTiltResult {
	// Transform to camera frame
	targetPositionInCameraFrame := utils.TransformPointToCameraFrame(cameraPose.Pose, targetPosition)

	cx := targetPositionInCameraFrame.X
	cy := targetPositionInCameraFrame.Y
	cz := targetPositionInCameraFrame.Z
	// Check if behind camera
	if cz < 0 {
		return utils.PanTiltResult{
			IsValid:      false,
			ErrorMessage: "Target is behind the camera",
		}
	}

	// Convert to spherical coordinates
	// Pan: positive pan = camera rotates right (toward +X), so pan = atan2(cx, cz)
	panRad := math.Atan2(cx, cz)
	rXZ := math.Sqrt(cx*cx + cz*cz)
	// Tilt: positive tilt = looking up = -Y in camera frame (upward-facing camera)
	tiltRad := math.Atan2(-cy, rXZ)

	// Convert to degrees
	panDeg := utils.RadiansToDegrees(panRad)
	tiltDeg := utils.RadiansToDegrees(tiltRad)

	// Wrap pan to [0, 360)
	if panDeg < 0 {
		panDeg += 360.0
	}

	// Check camera limits
	var errorMsg string
	isValid := true

	if panDeg < cameraPose.Limits.PanMinDeg || panDeg > cameraPose.Limits.PanMaxDeg {
		isValid = false
		errorMsg = fmt.Sprintf("Pan %.1f° outside range [%.1f°, %.1f°]",
			panDeg, cameraPose.Limits.PanMinDeg, cameraPose.Limits.PanMaxDeg)
	}

	if tiltDeg < cameraPose.Limits.TiltMinDeg || tiltDeg > cameraPose.Limits.TiltMaxDeg {
		isValid = false
		if errorMsg != "" {
			errorMsg += "; "
		}
		errorMsg += fmt.Sprintf("Tilt %.1f° outside range [%.1f°, %.1f°]",
			tiltDeg, cameraPose.Limits.TiltMinDeg, cameraPose.Limits.TiltMaxDeg)
	}

	// Convert to normalized
	panNorm := utils.DegreesToNormalized(panDeg, cameraPose.Limits.PanMinDeg, cameraPose.Limits.PanMaxDeg)
	tiltNorm := utils.DegreesToNormalized(tiltDeg, cameraPose.Limits.TiltMinDeg, cameraPose.Limits.TiltMaxDeg)

	// Clamp for safety
	panNorm = math.Max(-1.0, math.Min(1.0, panNorm))
	tiltNorm = math.Max(-1.0, math.Min(1.0, tiltNorm))

	return utils.PanTiltResult{
		PanNormalized:  panNorm,
		TiltNormalized: tiltNorm,
		PanDegrees:     panDeg,
		TiltDegrees:    tiltDeg,
		IsValid:        isValid,
		ErrorMessage:   errorMsg,
	}
}

type AbsolutePositionTracker struct {
	logger       logging.Logger
	cameraLimits utils.CameraLimits
	samples      []utils.PTZMeasurement
	cameraPose   *utils.CameraPose
}

const AbsolutePositionTrackerMinSamples = 6

func NewAbsolutePositionTracker(logger logging.Logger, cameraLimits utils.CameraLimits) (*AbsolutePositionTracker, error) {
	return &AbsolutePositionTracker{
		logger:       logger,
		cameraLimits: cameraLimits,
	}, nil
}

func (a *AbsolutePositionTracker) Calibrate(measurements []utils.PTZMeasurement) error {
	a.samples = measurements
	cameraPose, err := SolveCameraPose(a.samples, a.cameraLimits)
	if err != nil {
		return fmt.Errorf("failed to solve for camera pose: %w", err)
	}
	a.cameraPose = cameraPose
	// Store or use the cameraPose as needed
	// For example, you might want to save it to a file or use it in subsequent calculations
	return nil
}

func (a *AbsolutePositionTracker) CalculatePTZ(targetPose r3.Vector, cameraPose spatialmath.Pose) (utils.PTZValues, error) {
	ptzResult := XYZToPanTilt(targetPose, a.cameraPose)
	if !ptzResult.IsValid {
		return utils.PTZValues{}, fmt.Errorf("invalid PTZ result: %s", ptzResult.ErrorMessage)
	}
	zoom := utils.CalculateZoom(targetPose, cameraPose.Point(), a.cameraLimits)
	ptzValues := utils.PTZValues{
		Pan:  ptzResult.PanNormalized,
		Tilt: ptzResult.TiltNormalized,
		Zoom: zoom,
	}
	return ptzValues, nil
}

func (a *AbsolutePositionTracker) GetCalibration() (cameraPose spatialmath.Pose, err error) {
	if a.cameraPose == nil {
		return nil, fmt.Errorf("camera pose not yet calibrated")
	}
	return a.cameraPose.Pose, nil
}
