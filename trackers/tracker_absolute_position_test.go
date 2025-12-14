package trackers

import (
	"fmt"
	"math"
	"ptztargettracker/utils"
	"sort"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

var realSamples = []utils.PTZMeasurement{
	{
		Pan:  -0.03757746478873236,
		Tilt: 0.9544444444444444,
		TargetPosition: r3.Vector{
			X: -435.60399286176806,
			Y: 234.47770801179834,
			Z: -290.9939263146232,
		},
	},
	{
		Pan:  0.06552112676056332,
		Tilt: 0.9422222222222222,
		TargetPosition: r3.Vector{
			X: 0.011254228913583745,
			Y: 234.47002873097716,
			Z: -291.00260279499935,
		},
	},
	{
		Pan:  0.15177464788732398,
		Tilt: 1,
		TargetPosition: r3.Vector{
			X: 435.5789230503495,
			Y: 234.42667055111667,
			Z: -291.0174736475226,
		},
	},
	{
		Pan:  -0.03335211267605631,
		Tilt: 0.9486666666666665,
		TargetPosition: r3.Vector{
			X: -435.61611444258995,
			Y: 372.26205755273196,
			Z: -290.97589187287326,
		},
	},
	{
		Pan:  0.07881690140845077,
		Tilt: 0.9575555555555557,
		TargetPosition: r3.Vector{
			X: -0.01038746369999044,
			Y: 372.24416556153136,
			Z: -291.00586447709486,
		},
	},
	{
		Pan:  0.16692957746478876,
		Tilt: 1,
		TargetPosition: r3.Vector{
			X: 435.64087408821166,
			Y: 372.26621021806557,
			Z: -290.983085223046,
		},
	},
	{
		Pan:  -0.03284507042253514,
		Tilt: 0.956888888888889,
		TargetPosition: r3.Vector{
			X: -435.59932191336964,
			Y: 510.09457227836407,
			Z: -290.99815419812955,
		},
	},
	{
		Pan:  0.0900281690140845,
		Tilt: 0.9784444444444444,
		TargetPosition: r3.Vector{
			X: -0.021515586337596808,
			Y: 510.07716780288814,
			Z: -291.01070430730505,
		},
	},
	{
		Pan:  -0.0865915492957745,
		Tilt: 0.9651111111111111,
		TargetPosition: r3.Vector{
			X: -638.9820616804648,
			Y: 234.47634134025643,
			Z: -291.00749550798605,
		},
	},
	{
		Pan:  0.1785915492957746,
		Tilt: 1,
		TargetPosition: r3.Vector{
			X: 638.9880634644423,
			Y: 234.48606056269705,
			Z: -290.98832319802017,
		},
	},
	{
		Pan:  -0.03836619718309886,
		Tilt: 0.6711111111111112,
		TargetPosition: r3.Vector{
			X: -435.60455533031705,
			Y: 234.46733349748385,
			Z: 121.48554806379826,
		},
	},
	{
		Pan:  0.08642253521126764,
		Tilt: 0.6997777777777777,
		TargetPosition: r3.Vector{
			X: 0.0014103939345261802,
			Y: 234.46118014422143,
			Z: 121.4936976669821,
		},
	},
	{
		Pan:  0.1830422535211268,
		Tilt: 0.7944444444444444,
		TargetPosition: r3.Vector{
			X: 435.5924250037507,
			Y: 234.45231694107648,
			Z: 121.48677192885114,
		},
	},
	{
		Pan:  0.14636619718309873,
		Tilt: 0.6884444444444446,
		TargetPosition: r3.Vector{
			X: 0.007235611241796091,
			Y: 717.7758250470246,
			Z: 121.52061848256434,
		},
	},
	{
		Pan:  -0.11470422535211267,
		Tilt: 0.8457777777777777,
		TargetPosition: r3.Vector{
			X: -749.9566850901077,
			Y: 0.009489272637737578,
			Z: -96.0460903047465,
		},
	},
	{
		Pan:  0.19109859154929584,
		Tilt: 0.9377777777777778,
		TargetPosition: r3.Vector{
			X: 750.0243678703018,
			Y: 0.0071622319935342555,
			Z: -95.94294283597682,
		},
	},
	{
		Pan:  0.1312676056338027,
		Tilt: 0.8573333333333333,
		TargetPosition: r3.Vector{
			X: 0.014619968759572131,
			Y: 750.0063971801737,
			Z: -96.00659783821769,
		},
	},
}

func TestEndToEnd2(t *testing.T) {
	approxCameraPos := r3.Vector{X: -550.0, Y: 2000.0, Z: -150.0}
	limits := utils.CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  355.0,
		TiltMinDeg: 5.0,
		TiltMaxDeg: 90.0,
	}

	t.Logf("PTZ Camera Calibration")
	t.Logf("======================")
	t.Logf("Camera: Pan=[%.0f°,%.0f°], Tilt=[%.0f°,%.0f°]\n\n",
		limits.PanMinDeg, limits.PanMaxDeg, limits.TiltMinDeg, limits.TiltMaxDeg)

	utils.ValidateMeasurements(realSamples)

	t.Logf("\nSolving for camera pose...")
	cameraPose, err := SolveCameraPoseStandard(realSamples, limits, approxCameraPos)
	if err != nil {
		t.Errorf("Error: %v", err)
		return
	}

	// Print pose information
	point := cameraPose.Point()
	orientation := cameraPose.Orientation()

	// Try to get axis-angle representation
	ov := orientation.AxisAngles()
	t.Logf("\nCamera Pose:  Position: [%.3f, %.3f, %.3f], Orientation: [theta: %.3f rad (%.1f°), axis: [%.3f, %.3f, %.3f]]\n",
		point.X, point.Y, point.Z, ov.Theta, ov.Theta*180/math.Pi, ov.RX, ov.RY, ov.RZ)

	// You can also convert to other representations
	quat := orientation.Quaternion()
	t.Logf("  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]\n",
		quat.Real, quat.Imag, quat.Jmag, quat.Kmag)

	utils.ValidateCalibration(realSamples, cameraPose, limits, XYZToPanTiltStandard)

	// Example usage
	t.Logf("\n\nExample: Track new target")
	result := XYZToPanTiltStandard(r3.Vector{X: 100.0, Y: 200.0, Z: -50.0}, cameraPose, limits)
	t.Logf("Target: [100.000, 200.000, -50.000]\n")
	if result.IsValid {
		t.Logf("✓ Pan: %.3f, Tilt: %.3f (normalized)\n",
			result.PanNormalized, result.TiltNormalized)
		t.Logf("  (%.1f°, %.1f°)\n", result.PanDegrees, result.TiltDegrees)
	} else {
		t.Logf("✗ %s\n", result.ErrorMessage)
	}
}

func AnalyzePanTiltMapping(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits) {
	fmt.Println("Analyzing Pan/Tilt Mapping from Real Data")
	fmt.Println("==========================================")

	for i, sample := range samples {
		// Transform to camera frame (we know this works - Z>0 for all)
		camFrame := utils.TransformPointToCameraFrame(cameraPose, sample.TargetPosition)

		// Convert actual pan/tilt from normalized to degrees
		actualPanDeg := utils.NormalizedToDegrees(sample.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		actualTiltDeg := utils.NormalizedToDegrees(sample.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)

		// What does our current formula predict?
		predictedPanRad := math.Atan2(camFrame.X, camFrame.Z)
		predictedPanDeg := predictedPanRad * 180 / math.Pi
		if predictedPanDeg < 0 {
			predictedPanDeg += 360
		}

		rXZ := math.Sqrt(camFrame.X*camFrame.X + camFrame.Z*camFrame.Z)
		predictedTiltRad := math.Atan2(-camFrame.Y, rXZ)
		predictedTiltDeg := predictedTiltRad * 180 / math.Pi

		fmt.Printf("Sample %d:\n", i+1)
		fmt.Printf("  Camera frame: [%.1f, %.1f, %.1f]\n", camFrame.X, camFrame.Y, camFrame.Z)
		fmt.Printf("  Actual pan: %.1f°, Actual tilt: %.1f°\n", actualPanDeg, actualTiltDeg)
		fmt.Printf("  Predicted pan: %.1f°, Predicted tilt: %.1f°\n", predictedPanDeg, predictedTiltDeg)
		fmt.Printf("  Error: Pan %.1f°, Tilt %.1f°\n\n",
			actualPanDeg-predictedPanDeg, actualTiltDeg-predictedTiltDeg)
	}

	// Now let's try different formulas
	fmt.Println("\n\nTrying Alternative Formulas:")
	fmt.Println("================================")

	// Try: pan = atan2(-cx, cz) (negative X)
	fmt.Println("Formula 1: pan = atan2(-cx, cz)")
	testFormula(samples, cameraPose, limits, func(cx, cy, cz float64) (float64, float64) {
		pan := math.Atan2(-cx, cz)
		rXZ := math.Sqrt(cx*cx + cz*cz)
		tilt := math.Atan2(-cy, rXZ)
		return pan, tilt
	})

	// Try: pan = atan2(cx, -cz) (negative Z)
	fmt.Println("\nFormula 2: pan = atan2(cx, -cz)")
	testFormula(samples, cameraPose, limits, func(cx, cy, cz float64) (float64, float64) {
		pan := math.Atan2(cx, -cz)
		rXZ := math.Sqrt(cx*cx + cz*cz)
		tilt := math.Atan2(-cy, rXZ)
		return pan, tilt
	})

	// Try: tilt = atan2(cy, rXZ) (positive Y)
	fmt.Println("\nFormula 3: tilt = atan2(cy, rXZ)")
	testFormula(samples, cameraPose, limits, func(cx, cy, cz float64) (float64, float64) {
		pan := math.Atan2(cx, cz)
		rXZ := math.Sqrt(cx*cx + cz*cz)
		tilt := math.Atan2(cy, rXZ)
		return pan, tilt
	})
}

func testFormula(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits,
	formula func(cx, cy, cz float64) (panRad, tiltRad float64)) {

	var totalPanError, totalTiltError float64

	for _, sample := range samples {
		camFrame := utils.TransformPointToCameraFrame(cameraPose, sample.TargetPosition)

		panRad, tiltRad := formula(camFrame.X, camFrame.Y, camFrame.Z)

		panDeg := panRad * 180 / math.Pi
		if panDeg < 0 {
			panDeg += 360
		}
		tiltDeg := tiltRad * 180 / math.Pi

		actualPanDeg := utils.NormalizedToDegrees(sample.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		actualTiltDeg := utils.NormalizedToDegrees(sample.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)

		panError := math.Abs(actualPanDeg - panDeg)
		tiltError := math.Abs(actualTiltDeg - tiltDeg)

		totalPanError += panError
		totalTiltError += tiltError
	}

	fmt.Printf("  Avg Pan Error: %.1f°\n", totalPanError/float64(len(samples)))
	fmt.Printf("  Avg Tilt Error: %.1f°\n", totalTiltError/float64(len(samples)))
}

func CalculateOptimalOffset(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits) {
	fmt.Println("Calculating optimal pan offset for each sample:")
	fmt.Println("================================================")

	var offsets []float64

	for i, sample := range samples {
		camFrame := utils.TransformPointToCameraFrame(cameraPose, sample.TargetPosition)

		// Raw prediction
		panRad := math.Atan2(camFrame.X, camFrame.Z)
		predictedPanDeg := panRad * 180 / math.Pi
		if predictedPanDeg < 0 {
			predictedPanDeg += 360
		}

		// Actual
		actualPanDeg := utils.NormalizedToDegrees(sample.Pan, limits.PanMinDeg, limits.PanMaxDeg)

		// Calculate needed offset
		offset := actualPanDeg - predictedPanDeg

		// Handle wrap-around
		if offset > 180 {
			offset -= 360
		} else if offset < -180 {
			offset += 360
		}

		offsets = append(offsets, offset)

		fmt.Printf("Sample %2d: cx=%7.1f, cz=%7.1f, offset=%7.2f°\n",
			i+1, camFrame.X, camFrame.Z, offset)
	}

	// Calculate statistics
	var sum, sumSq float64
	for _, off := range offsets {
		sum += off
		sumSq += off * off
	}
	avg := sum / float64(len(offsets))
	variance := sumSq/float64(len(offsets)) - avg*avg
	stddev := math.Sqrt(variance)

	sort.Float64s(offsets)
	fmt.Printf("\nOffset Statistics:\n")
	fmt.Printf("  Average: %.2f°\n", avg)
	fmt.Printf("  Std Dev: %.2f°\n", stddev)
	fmt.Printf("  Range: [%.2f°, %.2f°]\n", offsets[0], offsets[len(offsets)-1])
}

func TestEndToEnd3(t *testing.T) {
	// This test can be used to analyze real data pan/tilt mapping
	// after solving for camera pose
	// For brevity, we won't repeat the full implementation here
	approxCameraPos := r3.Vector{X: -550.0, Y: 2000.0, Z: -150.0}
	limits := utils.CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  355.0,
		TiltMinDeg: 5.0,
		TiltMaxDeg: 90.0,
	}
	cameraPose, err := SolveCameraPoseStandard(realSamples, limits, approxCameraPos)
	if err != nil {
		t.Errorf("Error: %v", err)
		return
	}

	CalculateOptimalOffset(realSamples, cameraPose, limits)
	AnalyzePanTiltMapping(realSamples, cameraPose, limits)
	FitPanFormula(realSamples, cameraPose, limits)
	FitTiltFormula(realSamples, cameraPose, limits)
}

func FitPanFormula(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits) {
	fmt.Println("Analyzing pan formula systematically:")
	fmt.Println("=====================================")

	// Try: pan = atan2(ay*cx + by*cy, az*cz + bz*cy)
	// This allows for coupling between axes

	formulas := []struct {
		name string
		fn   func(cx, cy, cz float64) float64
	}{
		{"atan2(cx, cy)", func(cx, cy, cz float64) float64 { return math.Atan2(cx, cy) }},
		{"atan2(cx, -cy)", func(cx, cy, cz float64) float64 { return math.Atan2(cx, -cy) }},
		{"atan2(-cx, cy)", func(cx, cy, cz float64) float64 { return math.Atan2(-cx, cy) }},
		{"atan2(-cx, -cy)", func(cx, cy, cz float64) float64 { return math.Atan2(-cx, -cy) }},
		{"atan2(cy, cx)", func(cx, cy, cz float64) float64 { return math.Atan2(cy, cx) }},
		{"atan2(-cy, cx)", func(cx, cy, cz float64) float64 { return math.Atan2(-cy, cx) }},
	}

	for _, formula := range formulas {
		fmt.Printf("\nTrying: %s\n", formula.name)
		testPanFormula(samples, cameraPose, limits, formula.fn)
	}
}

func testPanFormula(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits,
	panFn func(cx, cy, cz float64) float64) {

	var totalError float64

	for _, sample := range samples {
		camFrame := utils.TransformPointToCameraFrame(cameraPose, sample.TargetPosition)

		panRad := panFn(camFrame.X, camFrame.Y, camFrame.Z)
		panDeg := panRad * 180 / math.Pi
		if panDeg < 0 {
			panDeg += 360
		}

		actualPanDeg := utils.NormalizedToDegrees(sample.Pan, limits.PanMinDeg, limits.PanMaxDeg)

		error := math.Abs(actualPanDeg - panDeg)
		if error > 180 {
			error = 360 - error
		}
		totalError += error
	}

	fmt.Printf("  Avg error: %.1f°\n", totalError/float64(len(samples)))
}

func FitTiltFormula(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits) {
	fmt.Println("\nAnalyzing tilt formula systematically:")
	fmt.Println("======================================")

	formulas := []struct {
		name string
		fn   func(cx, cy, cz float64) float64
	}{
		{"atan2(cz, rCYCX)", func(cx, cy, cz float64) float64 {
			rCYCX := math.Sqrt(cx*cx + cy*cy)
			return math.Atan2(cz, rCYCX)
		}},
		{"atan2(-cz, rCYCX)", func(cx, cy, cz float64) float64 {
			rCYCX := math.Sqrt(cx*cx + cy*cy)
			return math.Atan2(-cz, rCYCX)
		}},
		{"atan2(cz, cy)", func(cx, cy, cz float64) float64 {
			return math.Atan2(cz, cy)
		}},
		{"atan2(cz, -cy)", func(cx, cy, cz float64) float64 {
			return math.Atan2(cz, -cy)
		}},
		{"atan2(-cz, cy)", func(cx, cy, cz float64) float64 {
			return math.Atan2(-cz, cy)
		}},
		{"atan2(-cz, -cy)", func(cx, cy, cz float64) float64 {
			return math.Atan2(-cz, -cy)
		}},
		{"atan2(cy, cz)", func(cx, cy, cz float64) float64 {
			return math.Atan2(cy, cz)
		}},
		{"atan2(-cy, cz)", func(cx, cy, cz float64) float64 {
			return math.Atan2(-cy, cz)
		}},
	}

	for _, formula := range formulas {
		fmt.Printf("\nTrying: %s\n", formula.name)
		testTiltFormula(samples, cameraPose, limits, formula.fn)
	}
}

func testTiltFormula(samples []utils.PTZMeasurement, cameraPose spatialmath.Pose, limits utils.CameraLimits,
	tiltFn func(cx, cy, cz float64) float64) {

	var totalError float64

	for _, sample := range samples {
		camFrame := utils.TransformPointToCameraFrame(cameraPose, sample.TargetPosition)

		tiltRad := tiltFn(camFrame.X, camFrame.Y, camFrame.Z)
		tiltDeg := tiltRad * 180 / math.Pi

		actualTiltDeg := utils.NormalizedToDegrees(sample.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)

		error := math.Abs(actualTiltDeg - tiltDeg)
		totalError += error
	}

	fmt.Printf("  Avg error: %.1f°\n", totalError/float64(len(samples)))
}
