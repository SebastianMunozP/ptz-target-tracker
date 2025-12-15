package trackers

import (
	"math"
	"ptztargettracker/utils"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func TestEndToEndSyntheticData1(t *testing.T) {
	// STEP 1: Define TRUE camera pose in world frame
	trueCameraPos := r3.Vector{X: 0.0, Y: 0.0, Z: 0.0}

	// Camera looks down +Y axis in world frame
	// This means: world +Y → camera +Z (forward)
	// This is a 90° rotation around X-axis
	trueCameraOrientation := &spatialmath.Quaternion{
		Real: math.Cos(math.Pi / 4), // 90° / 2
		Imag: math.Sin(math.Pi / 4), // Around X-axis
		Jmag: 0,
		Kmag: 0,
	}
	trueCameraPose := spatialmath.NewPose(trueCameraPos, trueCameraOrientation)

	limits := utils.CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  360.0,
		TiltMinDeg: -90.0,
		TiltMaxDeg: 90.0,
	}

	// STEP 2: Generate synthetic measurements
	var measurements []utils.PTZMeasurement
	numPoints := 8
	radius := 1000.0

	for i := range numPoints {
		// Create target positions in world frame
		// Semicircle in XY plane (where camera is looking)
		angle := -math.Pi + (float64(i)/float64(numPoints-1))*(2*math.Pi)

		worldX := radius * math.Sin(angle)
		worldY := radius * math.Cos(angle) // Along camera's view direction
		worldZ := 0.0

		worldPos := r3.Vector{X: worldX, Y: worldY, Z: worldZ}

		// STEP 3: Transform to camera frame using TRUE pose
		camPos := utils.TransformPointToCameraFrame(trueCameraPose, worldPos)
		cx, cy, cz := camPos.Point().X, camPos.Point().Y, camPos.Point().Z

		t.Logf("World pos: (%.1f, %.1f, %.1f) → Camera frame: (%.1f, %.1f, %.1f)",
			worldX, worldY, worldZ, cx, cy, cz)

		// STEP 4: Compute pan/tilt using STANDARD formulas
		panRad := math.Atan2(cx, cz)
		rXZ := math.Sqrt(cx*cx + cz*cz)
		tiltRad := math.Atan2(-cy, rXZ)

		panDeg := panRad * 180 / math.Pi
		if panDeg < -180 {
			panDeg += 360
		} else if panDeg > 180 {
			panDeg -= 360
		}
		tiltDeg := tiltRad * 180 / math.Pi

		t.Logf("  → Pan: %.2f°, Tilt: %.2f°", panDeg, tiltDeg)

		// Store measurement
		measurement := utils.PTZMeasurement{
			Pan:            utils.DegreesToNormalized(panDeg, limits.PanMinDeg, limits.PanMaxDeg),
			Tilt:           utils.DegreesToNormalized(tiltDeg, limits.TiltMinDeg, limits.TiltMaxDeg),
			TargetPosition: worldPos,
		}
		measurements = append(measurements, measurement)
	}
	// Auto-discover best formulas
	pose, formulas, err := AutoDiscoverBestFormulas(measurements, limits, trueCameraPos)
	if err != nil {
		t.Fatalf("Failed to discover formulas: %v", err)
	}

	// Create calibration object
	calibration := CameraCalibration{
		Pose:     pose,
		Formulas: formulas,
	}

	// Validate
	for i, meas := range measurements {
		result := XYZToPanTiltWithFormulas(meas.TargetPosition, calibration, limits)

		origPanDeg := utils.NormalizedToDegrees(meas.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		origTiltDeg := utils.NormalizedToDegrees(meas.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)
		if limits.PanMinDeg >= 0 {
			// Limits are in [0, 360) style (e.g., [0, 355])
			// Normalize predicted angle to [0, 360)
			for origPanDeg < 0 {
				origPanDeg += 360
			}
			for origPanDeg >= 360 {
				origPanDeg -= 360
			}
		} else {
			// Limits are in [-180, 180) style (e.g., [-180, 180])
			// Normalize predicted angle to [-180, 180)
			for origPanDeg < -180 {
				origPanDeg += 360
			}
			for origPanDeg >= 180 {
				origPanDeg -= 360
			}
		}

		panErr := math.Abs(result.PanDegrees - origPanDeg)
		tiltErr := math.Abs(result.TiltDegrees - origTiltDeg)

		t.Logf("Point %d: Pan error: %.4f°, Tilt error: %.4f°", i, panErr, tiltErr)
	}

}

func vectorsAlmostEqual(v1, v2 r3.Vector, tol float64) bool {
	return math.Abs(v1.X-v2.X) < tol &&
		math.Abs(v1.Y-v2.Y) < tol &&
		math.Abs(v1.Z-v2.Z) < tol
}

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

func TestEndToEndRealData1(t *testing.T) {
	approxCameraPos := r3.Vector{X: -550.0, Y: 2000.0, Z: -150.0}
	limits := utils.CameraLimits{
		PanMinDeg:  0.0,
		PanMaxDeg:  355.0,
		TiltMinDeg: 5.0,
		TiltMaxDeg: 90.0,
	}

	// Auto-discover best formulas
	pose, formulas, err := AutoDiscoverBestFormulas(realSamples, limits, approxCameraPos)
	if err != nil {
		t.Fatalf("Failed to discover formulas: %v", err)
	}

	// Create calibration object
	calibration := CameraCalibration{
		Pose:     pose,
		Formulas: formulas,
	}

	// Validate
	for i, meas := range realSamples {
		result := XYZToPanTiltWithFormulas(meas.TargetPosition, calibration, limits)

		origPanDeg := utils.NormalizedToDegrees(meas.Pan, limits.PanMinDeg, limits.PanMaxDeg)
		origTiltDeg := utils.NormalizedToDegrees(meas.Tilt, limits.TiltMinDeg, limits.TiltMaxDeg)

		panErr := math.Abs(result.PanDegrees - origPanDeg)
		tiltErr := math.Abs(result.TiltDegrees - origTiltDeg)

		t.Logf("Point %d: Pan error: %.4f°, Tilt error: %.4f°", i, panErr, tiltErr)
	}
}
