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

// FormulaSet represents a specific pan/tilt formula combination
type FormulaSet struct {
	Name        string
	PanFormula  string
	TiltFormula string
	ComputePan  func(cx, cy, cz float64) float64
	ComputeTilt func(cx, cy, cz float64) float64
}

// GetAllFormulaSets returns all reasonable formula combinations to try
func GetAllFormulaSets() []FormulaSet {
	return []FormulaSet{
		// Standard XZ-plane formulas (camera looking down +Z)
		{
			Name:        "Standard (XZ-plane, Y-up)",
			PanFormula:  "atan2(cx, cz)",
			TiltFormula: "atan2(-cy, sqrt(cx²+cz²))",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(cx, cz)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				rXZ := math.Sqrt(cx*cx + cz*cz)
				return math.Atan2(-cy, rXZ)
			},
		},

		// Angled mount (cy as depth axis)
		{
			Name:        "Angled mount (cy-depth)",
			PanFormula:  "atan2(-cx, cy)",
			TiltFormula: "atan2(-cy, cz)",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(-cx, cy)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				return math.Atan2(-cy, cz)
			},
		},

		// Variant: positive cx, cy depth
		{
			Name:        "Variant (cx positive, cy-depth)",
			PanFormula:  "atan2(cx, cy)",
			TiltFormula: "atan2(-cy, cz)",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(cx, cy)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				return math.Atan2(-cy, cz)
			},
		},

		// XY-plane formulas
		{
			Name:        "XY-plane (cz-depth)",
			PanFormula:  "atan2(cx, cy)",
			TiltFormula: "atan2(-cz, sqrt(cx²+cy²))",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(cx, cy)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				rXY := math.Sqrt(cx*cx + cy*cy)
				return math.Atan2(-cz, rXY)
			},
		},

		// Negative X variants
		{
			Name:        "Negative X (XZ-plane)",
			PanFormula:  "atan2(-cx, cz)",
			TiltFormula: "atan2(-cy, sqrt(cx²+cz²))",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(-cx, cz)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				rXZ := math.Sqrt(cx*cx + cz*cz)
				return math.Atan2(-cy, rXZ)
			},
		},

		// Positive Y tilt
		{
			Name:        "Positive Y tilt (XZ-plane)",
			PanFormula:  "atan2(cx, cz)",
			TiltFormula: "atan2(cy, sqrt(cx²+cz²))",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(cx, cz)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				rXZ := math.Sqrt(cx*cx + cz*cz)
				return math.Atan2(cy, rXZ)
			},
		},

		// YZ-plane formulas
		{
			Name:        "YZ-plane (cx-depth)",
			PanFormula:  "atan2(cy, cz)",
			TiltFormula: "atan2(-cx, sqrt(cy²+cz²))",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(cy, cz)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				rYZ := math.Sqrt(cy*cy + cz*cz)
				return math.Atan2(-cx, rYZ)
			},
		},

		// Additional combinations with different sign patterns
		{
			Name:        "Variant 2 (neg X, cy-depth, pos tilt)",
			PanFormula:  "atan2(-cx, cy)",
			TiltFormula: "atan2(cy, cz)",
			ComputePan: func(cx, cy, cz float64) float64 {
				return math.Atan2(-cx, cy)
			},
			ComputeTilt: func(cx, cy, cz float64) float64 {
				return math.Atan2(cy, cz)
			},
		},
	}
}

// FormulaSpecificResiduals computes residuals using a specific formula set
type FormulaSpecificResiduals struct {
	Measurements    []utils.PTZMeasurement
	Limits          utils.CameraLimits
	ApproxCameraPos r3.Vector
	Formulas        FormulaSet
}

func (r *FormulaSpecificResiduals) Func(params []float64) float64 {
	residuals := r.Residuals(params)
	sum := 0.0
	for _, res := range residuals {
		sum += res * res
	}
	return sum
}

func (r *FormulaSpecificResiduals) Residuals(params []float64) []float64 {
	// params: [qw, qx, qy, qz]
	qw, qx, qy, qz := params[0], params[1], params[2], params[3]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := &spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(r.ApproxCameraPos, orientation)
	residuals := make([]float64, 0, 2*len(r.Measurements))

	for _, meas := range r.Measurements {
		worldPoint := meas.TargetPosition
		targetInCameraFrame := utils.TransformPointToCameraFrame(pose, worldPoint)
		camPoint := targetInCameraFrame.Point()

		cx, cy, cz := camPoint.X, camPoint.Y, camPoint.Z

		// Compute pan/tilt using THIS formula set
		predictedPanRad := r.Formulas.ComputePan(cx, cy, cz)
		predictedTiltRad := r.Formulas.ComputeTilt(cx, cy, cz)

		// Convert to degrees
		predictedPanDeg := utils.RadiansToDegrees(predictedPanRad)
		predictedPanDeg = utils.WrapUpAngleDeg(predictedPanDeg, r.Limits.PanMinDeg, r.Limits.PanMaxDeg)
		predictedTiltDeg := utils.RadiansToDegrees(predictedTiltRad)
		predictedTiltDeg = utils.WrapUpAngleDeg(predictedTiltDeg, r.Limits.TiltMinDeg, r.Limits.TiltMaxDeg)

		// Check camera limits
		outOfRange := false
		if predictedPanDeg < r.Limits.PanMinDeg || predictedPanDeg > r.Limits.PanMaxDeg {
			outOfRange = true
		}
		if predictedTiltDeg < r.Limits.TiltMinDeg || predictedTiltDeg > r.Limits.TiltMaxDeg {
			outOfRange = true
		}

		if outOfRange {
			residuals = append(residuals, 1000.0, 1000.0)
			continue
		}

		// Normal error calculation
		actualPanDeg := utils.NormalizedToDegrees(meas.Pan, r.Limits.PanMinDeg, r.Limits.PanMaxDeg)
		actualPanDeg = utils.WrapUpAngleDeg(actualPanDeg, r.Limits.PanMinDeg, r.Limits.PanMaxDeg)
		actualTiltDeg := utils.NormalizedToDegrees(meas.Tilt, r.Limits.TiltMinDeg, r.Limits.TiltMaxDeg)
		actualTiltDeg = utils.WrapUpAngleDeg(actualTiltDeg, r.Limits.TiltMinDeg, r.Limits.TiltMaxDeg)

		panError := actualPanDeg - predictedPanDeg
		tiltError := actualTiltDeg - predictedTiltDeg

		// Normalize errors
		panErrorNorm := panError / (r.Limits.PanMaxDeg - r.Limits.PanMinDeg)
		tiltErrorNorm := tiltError / (r.Limits.TiltMaxDeg - r.Limits.TiltMinDeg)

		residuals = append(residuals, panErrorNorm, tiltErrorNorm)
	}

	return residuals
}

// AutoDiscoverBestFormulas tries all formula combinations and picks the best
func AutoDiscoverBestFormulas(measurements []utils.PTZMeasurement, limits utils.CameraLimits,
	approxCameraPos r3.Vector) (spatialmath.Pose, FormulaSet, error) {

	fmt.Println("\n╔════════════════════════════════════════════════════════╗")
	fmt.Println("║     AUTO-DISCOVERING OPTIMAL PAN/TILT FORMULAS         ║")
	fmt.Println("╚════════════════════════════════════════════════════════╝")

	formulaSets := GetAllFormulaSets()

	type Result struct {
		Formulas   FormulaSet
		Pose       spatialmath.Pose
		FinalError float64
		Status     optimize.Status
	}

	var results []Result

	for i, formulas := range formulaSets {
		fmt.Printf("\n[%d/%d] Testing: %s\n", i+1, len(formulaSets), formulas.Name)
		fmt.Printf("  Pan:  %s\n", formulas.PanFormula)
		fmt.Printf("  Tilt: %s\n", formulas.TiltFormula)

		pose, finalError, status := optimizeWithFormulas(measurements, limits, approxCameraPos, formulas)

		fmt.Printf("  → Final error: %.6f\n", finalError)
		fmt.Printf("  → Status: %v\n", status)

		results = append(results, Result{
			Formulas:   formulas,
			Pose:       pose,
			FinalError: finalError,
			Status:     status,
		})
	}

	// Find best result
	bestIdx := 0
	bestError := results[0].FinalError
	for i, result := range results {
		if result.FinalError < bestError {
			bestError = result.FinalError
			bestIdx = i
		}
	}

	best := results[bestIdx]

	fmt.Println("\n╔════════════════════════════════════════════════════════╗")
	fmt.Println("║                  WINNER FOUND!                         ║")
	fmt.Println("╚════════════════════════════════════════════════════════╝")
	fmt.Printf("Name:        %s\n", best.Formulas.Name)
	fmt.Printf("Pan:         %s\n", best.Formulas.PanFormula)
	fmt.Printf("Tilt:        %s\n", best.Formulas.TiltFormula)
	fmt.Printf("Final error: %.6f\n", best.FinalError)
	fmt.Printf("Position:    [%.3f, %.3f, %.3f]\n",
		approxCameraPos.X, approxCameraPos.Y, approxCameraPos.Z)

	quat := best.Pose.Orientation().Quaternion()
	fmt.Printf("Orientation: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]\n",
		quat.Real, quat.Imag, quat.Jmag, quat.Kmag)

	return best.Pose, best.Formulas, nil
}

func optimizeWithFormulas(measurements []utils.PTZMeasurement, limits utils.CameraLimits,
	approxCameraPos r3.Vector, formulas FormulaSet) (spatialmath.Pose, float64, optimize.Status) {

	// Compute initial orientation guess
	var sumX, sumY, sumZ float64
	for _, meas := range measurements {
		sumX += meas.TargetPosition.X
		sumY += meas.TargetPosition.Y
		sumZ += meas.TargetPosition.Z
	}
	n := float64(len(measurements))
	targetCentroid := r3.Vector{X: sumX / n, Y: sumY / n, Z: sumZ / n}

	lookDirection := r3.Vector{
		X: targetCentroid.X - approxCameraPos.X,
		Y: targetCentroid.Y - approxCameraPos.Y,
		Z: targetCentroid.Z - approxCameraPos.Z,
	}
	lookDirection = lookDirection.Mul(1.0 / lookDirection.Norm())

	cameraDefaultZ := r3.Vector{X: 0, Y: 0, Z: 1}
	rotAxis := cameraDefaultZ.Cross(lookDirection)
	rotAxisNorm := rotAxis.Norm()

	var initialQuat *spatialmath.Quaternion
	if rotAxisNorm < 1e-6 {
		dot := cameraDefaultZ.Dot(lookDirection)
		if dot < 0 {
			initialQuat = &spatialmath.Quaternion{Real: 0, Imag: 1, Jmag: 0, Kmag: 0}
		} else {
			initialQuat = &spatialmath.Quaternion{Real: 1, Imag: 0, Jmag: 0, Kmag: 0}
		}
	} else {
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

	rf := &FormulaSpecificResiduals{
		Measurements:    measurements,
		Limits:          limits,
		ApproxCameraPos: approxCameraPos,
		Formulas:        formulas,
	}

	x0 := []float64{
		initialQuat.Real, initialQuat.Imag, initialQuat.Jmag, initialQuat.Kmag,
	}

	problem := optimize.Problem{
		Func: rf.Func,
	}

	settings := &optimize.Settings{
		FuncEvaluations: 50000,
		Converger: &optimize.FunctionConverge{
			Absolute: 1e-10,
			Relative: 1e-10,
		},
	}

	result, err := optimize.Minimize(problem, x0, settings, &optimize.NelderMead{})
	if err != nil {
		return nil, math.Inf(1), optimize.Failure
	}

	finalError := rf.Func(result.X)

	// Reconstruct pose
	qw, qx, qy, qz := result.X[0], result.X[1], result.X[2], result.X[3]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := &spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(approxCameraPos, orientation)

	return pose, finalError, result.Status
}

// CameraCalibration stores both pose and formulas
type CameraCalibration struct {
	Pose     spatialmath.Pose
	Formulas FormulaSet
}

// XYZToPanTiltWithFormulas converts world position to pan/tilt using discovered formulas
func XYZToPanTiltWithFormulas(targetPosition r3.Vector, calibration CameraCalibration,
	cameraLimits utils.CameraLimits) utils.PanTiltResult {

	// Transform to camera frame
	targetInCameraFrame := utils.TransformPointToCameraFrame(calibration.Pose, targetPosition)
	camPoint := targetInCameraFrame.Point()
	cx, cy, cz := camPoint.X, camPoint.Y, camPoint.Z

	// Use discovered formulas
	panRad := calibration.Formulas.ComputePan(cx, cy, cz)
	tiltRad := calibration.Formulas.ComputeTilt(cx, cy, cz)

	// Convert to degrees
	panDeg := utils.RadiansToDegrees(panRad)
	panDeg = utils.WrapUpAngleDeg(panDeg, cameraLimits.PanMinDeg, cameraLimits.PanMaxDeg)
	tiltDeg := utils.RadiansToDegrees(tiltRad)
	tiltDeg = utils.WrapUpAngleDeg(tiltDeg, cameraLimits.TiltMinDeg, cameraLimits.TiltMaxDeg)

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

// var lastOrientationPrinted = spatialmath.EulerAngles{}

func (r *StandardCameraFrameResiduals) Residuals(params []float64) []float64 {
	// params: [tx, ty, tz, qw, qx, qy, qz]
	// Note: We optimize quaternion components directly (normalized later)

	// translation := r3.Vector{X: params[0], Y: params[1], Z: params[2]}

	// Create quaternion from params (will be normalized)
	qw, qx, qy, qz := params[0], params[1], params[2], params[3]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(r.ApproxCameraPos, &orientation)

	residuals := make([]float64, 0, 2*len(r.Measurements)+3)

	for _, meas := range r.Measurements {
		worldPoint := meas.TargetPosition
		// fmt.Printf(" Target point in world frame: %+v\n", worldPoint)
		targetInCameraFrame := utils.TransformPointToCameraFrame(pose, worldPoint)
		// fmt.Printf(" Target point in camera frame: %+v, camera pose: %+v\n", targetInCameraFrame, pose)
		camPoint := targetInCameraFrame.Point()

		// Compute pan/tilt
		predictedPanRad, predictedTiltRad := utils.CalculatePanTiltInCameraFrame(camPoint)

		// Convert to degrees
		predictedPanDeg := utils.RadiansToDegrees(predictedPanRad)
		predictedPanDeg = utils.WrapUpAngleDeg(predictedPanDeg, r.Limits.PanMinDeg, r.Limits.PanMaxDeg)
		predictedTiltDeg := utils.RadiansToDegrees(predictedTiltRad)
		predictedTiltDeg = utils.WrapUpAngleDeg(predictedTiltDeg, r.Limits.TiltMinDeg, r.Limits.TiltMaxDeg)

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
		// approxCameraPos.X, approxCameraPos.Y, approxCameraPos.Z, --- IGNORE ---
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

	// result, err := optimize.Minimize(problem, x0, settings, &optimize.NelderMead{})
	result, err := optimize.Minimize(problem, x0, settings, &optimize.CmaEsChol{})
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
	// translation := r3.Vector{
	// 	X: result.X[0],
	// 	Y: result.X[1],
	// 	Z: result.X[2],
	// }

	// Normalize quaternion
	qw, qx, qy, qz := result.X[0], result.X[1], result.X[2], result.X[3]
	qNorm := math.Sqrt(qw*qw + qx*qx + qy*qy + qz*qz)

	orientation := &spatialmath.Quaternion{
		Real: qw / qNorm,
		Imag: qx / qNorm,
		Jmag: qy / qNorm,
		Kmag: qz / qNorm,
	}

	pose := spatialmath.NewPose(approxCameraPos, orientation)

	fmt.Printf("\nSolved camera pose:\n")
	fmt.Printf("  Position: [%.3f, %.3f, %.3f]\n", approxCameraPos.X, approxCameraPos.Y, approxCameraPos.Z)
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

	panRad, tiltRad := utils.CalculatePanTiltInCameraFrame(camPoint)

	// Convert to degrees
	panDeg := panRad * 180 / math.Pi
	panDeg = utils.WrapUpAngleDeg(panDeg, cameraLimits.PanMinDeg, cameraLimits.PanMaxDeg)

	tiltDeg := tiltRad * 180 / math.Pi
	tiltDeg = utils.WrapUpAngleDeg(tiltDeg, cameraLimits.TiltMinDeg, cameraLimits.TiltMaxDeg)

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
