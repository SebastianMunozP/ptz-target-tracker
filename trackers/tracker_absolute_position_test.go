package trackers

import (
	"math"
	"ptztargettracker/utils"
	"testing"

	"github.com/golang/geo/r3"
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
