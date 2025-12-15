package trackers

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func TestCameraCalibrationSerializationRoundtrip(t *testing.T) {
	// Build a sample pose
	pose := spatialmath.NewPose(r3.Vector{X: 1.0, Y: 2.0, Z: 3.0}, &spatialmath.Quaternion{Real: 0.9238795, Imag: 0.3826834, Jmag: 0, Kmag: 0})

	for _, fs := range GetAllFormulaSets() {
		cam := CameraCalibration{
			Pose:     pose,
			Formulas: fs,
		}

		abs := CameraCalibrationToAbsolute(cam)

		reh, err := AbsoluteToCameraCalibration(&abs)
		if err != nil {
			t.Fatalf("failed to rehydrate formula set '%s': %v", fs.Names.Name, err)
		}

		if reh.Formulas.Names.Name != fs.Names.Name {
			t.Fatalf("formula name mismatch: got '%s', want '%s'", reh.Formulas.Names.Name, fs.Names.Name)
		}
		if reh.Formulas.Names.PanFormula != fs.Names.PanFormula || reh.Formulas.Names.TiltFormula != fs.Names.TiltFormula {
			t.Fatalf("formula strings mismatch for '%s'", fs.Names.Name)
		}
	}
}
