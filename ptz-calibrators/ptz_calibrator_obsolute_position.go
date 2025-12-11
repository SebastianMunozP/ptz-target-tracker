package ptzcalibrators

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/optimize"
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

// ResidualFunction for optimization
type ResidualFunction struct {
	Measurements []PTZMeasurement
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
		worldPoint := r3.Vector{X: meas.X, Y: meas.Y, Z: meas.Z}

		// Transform world point to camera frame
		camPoint := TransformPointToCameraFrame(pose, worldPoint)

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
func SolveCameraPose(measurements []PTZMeasurement, limits CameraLimits) (*CameraPose, error) {
	// Convert normalized pan/tilt to direction vectors
	directions := make([]r3.Vector, len(measurements))
	for i, meas := range measurements {
		panRad := NormalizedToRadians(meas.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		tiltRad := NormalizedToRadians(meas.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)
		directions[i] = PanTiltToDirection(panRad, tiltRad)
	}

	rf := &ResidualFunction{
		Measurements: measurements,
		Directions:   directions,
	}

	// Compute better initial guess: centroid of measurements
	var sumX, sumY, sumZ float64
	for _, meas := range measurements {
		sumX += meas.X
		sumY += meas.Y
		sumZ += meas.Z
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

	return &CameraPose{
		Pose:   pose,
		Limits: limits,
	}, nil
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

// XYZToPanTilt converts world coordinates to normalized pan/tilt
func XYZToPanTilt(x, y, z float64, cameraPose *CameraPose) PanTiltResult {
	// World point
	worldPoint := r3.Vector{X: x, Y: y, Z: z}

	// Transform to camera frame
	camPoint := TransformPointToCameraFrame(cameraPose.Pose, worldPoint)

	cx := camPoint.X
	cy := camPoint.Y
	cz := camPoint.Z

	// Check if behind camera
	if cz < 0 {
		return PanTiltResult{
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
	panDeg := RadiansToDegrees(panRad)
	tiltDeg := RadiansToDegrees(tiltRad)

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
	panNorm := DegreesToNormalized(panDeg, cameraPose.Limits.PanMinDeg, cameraPose.Limits.PanMaxDeg)
	tiltNorm := DegreesToNormalized(tiltDeg, cameraPose.Limits.TiltMinDeg, cameraPose.Limits.TiltMaxDeg)

	// Clamp for safety
	panNorm = math.Max(-1.0, math.Min(1.0, panNorm))
	tiltNorm = math.Max(-1.0, math.Min(1.0, tiltNorm))

	return PanTiltResult{
		PanNormalized:  panNorm,
		TiltNormalized: tiltNorm,
		PanDegrees:     panDeg,
		TiltDegrees:    tiltDeg,
		IsValid:        isValid,
		ErrorMessage:   errorMsg,
	}
}

// ValidateCalibration checks quality of measurements
func ValidateCalibration(measurements []PTZMeasurement) {
	n := len(measurements)
	if n < 3 {
		fmt.Println("⚠️  Warning: Less than 3 measurements (minimum required)")
		return
	}

	// Compute centroid
	var cx, cy, cz float64
	for _, m := range measurements {
		cx += m.X
		cy += m.Y
		cz += m.Z
	}
	cx /= float64(n)
	cy /= float64(n)
	cz /= float64(n)

	// Compute standard deviations
	var stdX, stdY, stdZ float64
	for _, m := range measurements {
		stdX += (m.X - cx) * (m.X - cx)
		stdY += (m.Y - cy) * (m.Y - cy)
		stdZ += (m.Z - cz) * (m.Z - cz)
	}
	stdX = math.Sqrt(stdX / float64(n))
	stdY = math.Sqrt(stdY / float64(n))
	stdZ = math.Sqrt(stdZ / float64(n))

	fmt.Printf("Calibration Quality:\n")
	fmt.Printf("  Points: %d\n", n)
	fmt.Printf("  Spread: X=%.3f, Y=%.3f, Z=%.3f\n", stdX, stdY, stdZ)

	// Depth variation
	var depthVar float64
	for _, m := range measurements {
		dx := m.X - cx
		dy := m.Y - cy
		dz := m.Z - cz
		depthVar += (dx*dx + dy*dy + dz*dz)
	}
	depthStd := math.Sqrt(depthVar / float64(n))
	fmt.Printf("  Depth variation: %.3f\n", depthStd)

	if stdX < 0.1 || stdY < 0.1 || stdZ < 0.1 {
		fmt.Println("  ⚠️  Low spatial spread - points too clustered")
	}
	if depthStd < 0.1 {
		fmt.Println("  ⚠️  Low depth variation")
	}
}

// TestCalibration validates solved pose
func TestCalibration(measurements []PTZMeasurement, cameraPose *CameraPose) {
	fmt.Println("\nCalibration Validation:")
	fmt.Println("Point | Pan Error (°) | Tilt Error (°) | Status")
	fmt.Println("------|---------------|----------------|--------")

	var totalPanErr, totalTiltErr float64
	validCount := 0

	for i, m := range measurements {
		result := XYZToPanTilt(m.X, m.Y, m.Z, cameraPose)

		panOrigDeg := NormalizedToDegrees(m.Pan, cameraPose.Limits.PanMinDeg, cameraPose.Limits.PanMaxDeg)
		tiltOrigDeg := NormalizedToDegrees(m.Tilt, cameraPose.Limits.TiltMinDeg, cameraPose.Limits.TiltMaxDeg)

		panErr := result.PanDegrees - panOrigDeg
		tiltErr := result.TiltDegrees - tiltOrigDeg

		// Normalize pan error to [-180, 180] to handle wrapping
		for panErr > 180 {
			panErr -= 360
		}
		for panErr < -180 {
			panErr += 360
		}

		status := "✓"
		if result.IsValid {
			totalPanErr += math.Abs(panErr)
			totalTiltErr += math.Abs(tiltErr)
			validCount++
		} else {
			status = "✗"
		}

		fmt.Printf("%5d | %13.3f | %14.3f | %s\n", i+1, panErr, tiltErr, status)
	}

	if validCount > 0 {
		fmt.Printf("\nAvg errors: Pan=%.3f°, Tilt=%.3f°\n",
			totalPanErr/float64(validCount),
			totalTiltErr/float64(validCount))
	}
}

func main() {
	// Example calibration measurements
	measurements := []PTZMeasurement{
		{Pan: 0.3, Tilt: -0.2, X: 1.0, Y: 0.5, Z: 2.0},
		{Pan: -0.3, Tilt: -0.2, X: -1.0, Y: 0.5, Z: 2.0},
		{Pan: 0.3, Tilt: 0.15, X: 1.0, Y: -0.5, Z: 2.0},
		{Pan: -0.3, Tilt: 0.15, X: -1.0, Y: -0.5, Z: 2.0},
		{Pan: 0.0, Tilt: 0.0, X: 0.0, Y: 0.0, Z: 1.5},
		{Pan: 0.1, Tilt: -0.25, X: 0.5, Y: 0.8, Z: 3.0},
		{Pan: -0.1, Tilt: 0.25, X: -0.5, Y: -0.8, Z: 3.0},
		{Pan: 0.4, Tilt: 0.0, X: 1.5, Y: 0.0, Z: 2.5},
	}

	limits := DefaultCameraLimits

	fmt.Println("PTZ Camera Calibration")
	fmt.Println("======================")
	fmt.Printf("Camera: Pan=[%.0f°,%.0f°], Tilt=[%.0f°,%.0f°]\n\n",
		limits.PanMinDeg, limits.PanMaxDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

	ValidateCalibration(measurements)

	fmt.Println("\nSolving for camera pose...")
	cameraPose, err := SolveCameraPose(measurements, limits)
	if err != nil {
		fmt.Printf("Error: %v\n", err)
		return
	}

	// Print pose information
	fmt.Printf("\nCamera Pose:\n")
	point := cameraPose.Pose.Point()
	fmt.Printf("  Position: [%.3f, %.3f, %.3f]\n", point.X, point.Y, point.Z)

	orientation := cameraPose.Pose.Orientation()
	if ov, ok := orientation.(*spatialmath.OrientationVector); ok {
		fmt.Printf("  Orientation (axis-angle): θ=%.3f, axis=[%.3f, %.3f, %.3f]\n",
			ov.Theta, ov.OX, ov.OY, ov.OZ)
	}

	// You can also convert to other representations
	quat := orientation.Quaternion()
	fmt.Printf("  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]\n",
		quat.Real, quat.Imag, quat.Jmag, quat.Kmag)

	TestCalibration(measurements, cameraPose)

	// Example usage
	fmt.Println("\n\nExample: Track new target")
	result := XYZToPanTilt(0.8, 0.3, 2.2, cameraPose)
	fmt.Printf("Target: [0.800, 0.300, 2.200]\n")
	if result.IsValid {
		fmt.Printf("✓ Pan: %.3f, Tilt: %.3f (normalized)\n",
			result.PanNormalized, result.TiltNormalized)
		fmt.Printf("  (%.1f°, %.1f°)\n", result.PanDegrees, result.TiltDegrees)
	} else {
		fmt.Printf("✗ %s\n", result.ErrorMessage)
	}
}
