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

// StandardCameraFrameResiduals enforces a fixed camera frame convention:
// - pan = atan2(cx, cz), where positive pan = camera rotates right
// - tilt = atan2(cy, rXZ), where positive tilt = camera tilts up
// The camera pose is optimized to make these formulas work
type StandardCameraFrameResiduals struct {
	Measurements        []utils.PTZMeasurement
	Limits              utils.CameraLimits
	ApproxCameraPos     r3.Vector
	PositionUncertainty float64
}

func (r *StandardCameraFrameResiduals) Func(params []float64) float64 {
	residuals := r.Residuals(params)
	sum := 0.0
	for _, res := range residuals {
		sum += res * res
	}
	return sum
}

func (r *StandardCameraFrameResiduals) Residuals(params []float64) []float64 {
	// params: [tx, ty, tz, qw, qx, qy, qz]
	// Note: We optimize quaternion components directly (normalized later)

	translation := r3.Vector{X: params[0], Y: params[1], Z: params[2]}

	// Create quaternion from params (will be normalized)
	qw, qx, qy, qz := params[3], params[4], params[5], params[6]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := &spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(translation, orientation)

	residuals := make([]float64, 0, 2*len(r.Measurements)+3)

	for _, meas := range r.Measurements {
		worldPoint := meas.TargetPosition
		camPose := utils.TransformPointToCameraFrame(pose, worldPoint)
		camPoint := camPose.Point()
		cx, cy, cz := camPoint.X, camPoint.Y, camPoint.Z

		if cz <= 0 {
			residuals = append(residuals, 1000.0, 1000.0)
			continue
		}

		// Compute pan/tilt
		predictedPanRad := math.Atan2(cx, cz)
		predictedPanDeg := predictedPanRad * 180 / math.Pi
		if predictedPanDeg < 0 {
			predictedPanDeg += 360
		}

		rXZ := math.Sqrt(cx*cx + cz*cz)
		predictedTiltRad := math.Atan2(-cy, rXZ)
		predictedTiltDeg := predictedTiltRad * 180 / math.Pi

		// CHECK CAMERA LIMITS
		outOfRange := false

		// Pan must be in [PanMinDeg, PanMaxDeg]
		if predictedPanDeg < r.Limits.PanMinDeg || predictedPanDeg > r.Limits.PanMaxDeg {
			outOfRange = true
		}

		// Tilt must be in [TiltMinDeg, TiltMaxDeg]
		if predictedTiltDeg < r.Limits.TiltMinDeg || predictedTiltDeg > r.Limits.TiltMaxDeg {
			outOfRange = true
		}

		// If out of range, add large penalty
		if outOfRange {
			residuals = append(residuals, 1000.0, 1000.0)
			continue
		}

		// Normal error calculation
		actualPanDeg := utils.NormalizedToDegrees(meas.Pan, r.Limits.PanMinDeg, r.Limits.PanMaxDeg)
		actualTiltDeg := utils.NormalizedToDegrees(meas.Tilt, r.Limits.TiltMinDeg, r.Limits.TiltMaxDeg)

		panError := actualPanDeg - predictedPanDeg
		if panError > 180 {
			panError -= 360
		} else if panError < -180 {
			panError += 360
		}

		tiltError := actualTiltDeg - predictedTiltDeg

		panErrorNorm := panError / (r.Limits.PanMaxDeg - r.Limits.PanMinDeg)
		tiltErrorNorm := tiltError / (r.Limits.TiltMaxDeg - r.Limits.TiltMinDeg)

		residuals = append(residuals, panErrorNorm, tiltErrorNorm)
	}

	// Soft position constraint
	weight := 1.0 / (r.PositionUncertainty * r.PositionUncertainty)
	posError := r3.Vector{
		X: weight * (params[0] - r.ApproxCameraPos.X),
		Y: weight * (params[1] - r.ApproxCameraPos.Y),
		Z: weight * (params[2] - r.ApproxCameraPos.Z),
	}

	residuals = append(residuals, posError.X, posError.Y, posError.Z)

	return residuals
}

// SolveCameraPoseStandard solves for camera pose using FIXED standard formulas
// This approach enforces: pan = atan2(cx, cz), tilt = atan2(cy, rXZ)
// The camera orientation automatically compensates for mounting and manufacturer quirks
func SolveCameraPoseStandard(measurements []utils.PTZMeasurement, limits utils.CameraLimits,
	approxCameraPos r3.Vector) (spatialmath.Pose, error) {

	fmt.Printf("Solving camera pose with STANDARD formulas:\n")
	fmt.Printf("  pan = atan2(cx, cz)\n")
	fmt.Printf("  tilt = atan2(-cy, sqrt(cx²+cz²))\n\n")

	// Compute target centroid for initial orientation guess
	var sumX, sumY, sumZ float64
	for _, meas := range measurements {
		sumX += meas.TargetPosition.X
		sumY += meas.TargetPosition.Y
		sumZ += meas.TargetPosition.Z
	}
	n := float64(len(measurements))
	targetCentroid := r3.Vector{X: sumX / n, Y: sumY / n, Z: sumZ / n}

	// Initial orientation: point camera toward target centroid
	lookDirection := r3.Vector{
		X: targetCentroid.X - approxCameraPos.X,
		Y: targetCentroid.Y - approxCameraPos.Y,
		Z: targetCentroid.Z - approxCameraPos.Z,
	}
	lookDirection = lookDirection.Mul(1.0 / lookDirection.Norm())

	// Camera default points along +Z
	cameraDefaultZ := r3.Vector{X: 0, Y: 0, Z: 1}

	// Compute rotation to align camera with look direction
	rotAxis := cameraDefaultZ.Cross(lookDirection)
	rotAxisNorm := rotAxis.Norm()

	var initialQuat *spatialmath.Quaternion
	if rotAxisNorm < 1e-6 {
		// Parallel or anti-parallel
		dot := cameraDefaultZ.Dot(lookDirection)
		if dot < 0 {
			// 180° rotation around X
			initialQuat = &spatialmath.Quaternion{Real: 0, Imag: 1, Jmag: 0, Kmag: 0}
		} else {
			// Identity
			initialQuat = &spatialmath.Quaternion{Real: 1, Imag: 0, Jmag: 0, Kmag: 0}
		}
	} else {
		// General rotation
		rotAxis = rotAxis.Mul(1.0 / rotAxisNorm)
		dot := cameraDefaultZ.Dot(lookDirection)
		angle := math.Acos(utils.Clamp(dot, -1.0, 1.0))

		halfAngle := angle / 2
		sinHalf := math.Sin(halfAngle)

		initialQuat = &spatialmath.Quaternion{
			Real: math.Cos(halfAngle),
			Imag: rotAxis.X * sinHalf,
			Jmag: rotAxis.Y * sinHalf,
			Kmag: rotAxis.Z * sinHalf,
		}
	}

	rf := &StandardCameraFrameResiduals{
		Measurements:        measurements,
		Limits:              limits,
		ApproxCameraPos:     approxCameraPos,
		PositionUncertainty: 1000, // 1 meter uncertainty in millimeters
	}

	// Initial parameters: [tx, ty, tz, qw, qx, qy, qz]
	x0 := []float64{
		approxCameraPos.X, approxCameraPos.Y, approxCameraPos.Z,
		initialQuat.Real, initialQuat.Imag, initialQuat.Jmag, initialQuat.Kmag,
	}

	problem := optimize.Problem{
		Func: rf.Func,
	}

	settings := &optimize.Settings{
		FuncEvaluations: 100000,
		Converger: &optimize.FunctionConverge{
			Absolute: 1e-10,
			Relative: 1e-10,
		},
	}

	result, err := optimize.Minimize(problem, x0, settings, &optimize.NelderMead{})
	if err != nil {
		return nil, fmt.Errorf("optimization failed: %w", err)
	}

	// Evaluate final error
	finalResidual := rf.Func(result.X)
	avgErrorPerSample := finalResidual / float64(len(measurements))

	fmt.Printf("Optimization complete:\n")
	fmt.Printf("  Final residual: %.6f\n", finalResidual)
	fmt.Printf("  Avg error per sample: %.6f\n", avgErrorPerSample)
	fmt.Printf("  Function evaluations: %d\n", result.Stats.FuncEvaluations)
	fmt.Printf("  Status: %v\n", result.Status)

	// Extract optimized pose
	translation := r3.Vector{
		X: result.X[0],
		Y: result.X[1],
		Z: result.X[2],
	}

	// Normalize quaternion
	qw, qx, qy, qz := result.X[3], result.X[4], result.X[5], result.X[6]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := &spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(translation, orientation)

	fmt.Printf("\nSolved camera pose:\n")
	fmt.Printf("  Position: [%.3f, %.3f, %.3f]\n", translation.X, translation.Y, translation.Z)
	fmt.Printf("  Orientation (quat): [w=%.3f, x=%.3f, y=%.3f, z=%.3f]\n",
		orientation.Real, orientation.Imag, orientation.Jmag, orientation.Kmag)

	if avgErrorPerSample > 1.0 {
		return nil, fmt.Errorf("calibration error too large (avg %.3f per sample)", avgErrorPerSample)
	}

	return pose, nil
}

// XYZToPanTiltStandard converts world position to pan/tilt using FIXED standard formulas
func XYZToPanTiltStandard(targetPosition r3.Vector, cameraPose spatialmath.Pose,
	cameraLimits utils.CameraLimits) utils.PanTiltResult {

	// Transform to camera frame
	camPose := utils.TransformPointToCameraFrame(cameraPose, targetPosition)
	camPoint := camPose.Point()

	// Check if behind camera
	if camPoint.Z <= 0 {
		return utils.PanTiltResult{
			IsValid:      false,
			ErrorMessage: "Target is behind the camera",
		}
	}

	panRad, tiltRad := utils.CalculatePanTiltInCameraFrame(camPoint)

	// Convert to degrees
	panDeg := panRad * 180 / math.Pi
	if panDeg < 0 {
		panDeg += 360
	}
	tiltDeg := tiltRad * 180 / math.Pi

	// Validate against camera limits
	var errorMsg string
	isValid := true

	if panDeg < cameraLimits.PanMinDeg || panDeg > cameraLimits.PanMaxDeg {
		isValid = false
		errorMsg = fmt.Sprintf("Pan %.1f° outside range [%.1f°, %.1f°]",
			panDeg, cameraLimits.PanMinDeg, cameraLimits.PanMaxDeg)
	}

	if tiltDeg < cameraLimits.TiltMinDeg || tiltDeg > cameraLimits.TiltMaxDeg {
		isValid = false
		if errorMsg != "" {
			errorMsg += "; "
		}
		errorMsg += fmt.Sprintf("Tilt %.1f° outside range [%.1f°, %.1f°]",
			tiltDeg, cameraLimits.TiltMinDeg, cameraLimits.TiltMaxDeg)
	}

	// Convert to normalized
	panNorm := utils.DegreesToNormalized(panDeg, cameraLimits.PanMinDeg, cameraLimits.PanMaxDeg)
	tiltNorm := utils.DegreesToNormalized(tiltDeg, cameraLimits.TiltMinDeg, cameraLimits.TiltMaxDeg)

	// Clamp for safety
	panNorm = utils.Clamp(panNorm, cameraLimits.PanMinNormalized, cameraLimits.PanMaxNormalized)
	tiltNorm = utils.Clamp(tiltNorm, cameraLimits.TiltMinNormalized, cameraLimits.TiltMaxNormalized)

	return utils.PanTiltResult{
		PanNormalized:  panNorm,
		TiltNormalized: tiltNorm,
		PanDegrees:     panDeg,
		TiltDegrees:    tiltDeg,
		IsValid:        isValid,
		ErrorMessage:   errorMsg,
	}
}

// AbsolutePositionCalibration stores camera pose for JSON serialization
type AbsolutePositionCalibration struct {
	Translation struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
		Z float64 `json:"z"`
	} `json:"translation"`
	Orientation struct {
		W float64 `json:"w"` // Real component
		X float64 `json:"x"` // Imag
		Y float64 `json:"y"` // Jmag
		Z float64 `json:"z"` // Kmag
	} `json:"orientation"`
}

// ToPose converts calibration data to spatialmath.Pose
func (c *AbsolutePositionCalibration) ToPose() spatialmath.Pose {
	translation := r3.Vector{
		X: c.Translation.X,
		Y: c.Translation.Y,
		Z: c.Translation.Z,
	}

	orientation := &spatialmath.Quaternion{
		Real: c.Orientation.W,
		Imag: c.Orientation.X,
		Jmag: c.Orientation.Y,
		Kmag: c.Orientation.Z,
	}

	return spatialmath.NewPose(translation, orientation)
}

type AbsolutePositionTracker struct {
	logger       logging.Logger
	cameraLimits utils.CameraLimits
	samples      []utils.PTZMeasurement
	cameraPose   spatialmath.Pose
}

const AbsolutePositionTrackerMinSamples = 6

func NewAbsolutePositionTracker(logger logging.Logger, cameraLimits utils.CameraLimits, cameraPose spatialmath.Pose) (*AbsolutePositionTracker, error) {
	return &AbsolutePositionTracker{
		logger:       logger,
		cameraLimits: cameraLimits,
		cameraPose:   cameraPose,
	}, nil
}

func NewAbsolutePositionTrackerWithCalibration(logger logging.Logger, cameraLimits utils.CameraLimits, calibration *AbsolutePositionCalibration) (*AbsolutePositionTracker, error) {
	return &AbsolutePositionTracker{
		logger:       logger,
		cameraLimits: cameraLimits,
		cameraPose:   calibration.ToPose(),
	}, nil
}

func (a *AbsolutePositionTracker) Calibrate(measurements []utils.PTZMeasurement) error {
	a.samples = measurements
	// Approximate camera position (your guesstimate)
	cameraPose, err := SolveCameraPoseStandard(a.samples, a.cameraLimits, a.cameraPose.Point())
	if err != nil {
		return fmt.Errorf("failed to solve for camera pose: %w", err)
	}

	a.cameraPose = cameraPose
	return nil
}

func (a *AbsolutePositionTracker) CalculatePTZ(targetPose r3.Vector, cameraPose spatialmath.Pose) (utils.PTZValues, error) {
	fmt.Printf("cameraPose: %.3f,%.3f,%.3f, a.cameraPose: %.3f,%.3f,%.3f\n", cameraPose.Point().X, cameraPose.Point().Y, cameraPose.Point().Z, a.cameraPose.Point().X, a.cameraPose.Point().Y, a.cameraPose.Point().Z)
	ptzResult := XYZToPanTiltStandard(targetPose, cameraPose, a.cameraLimits)
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
	return a.cameraPose, nil
}
