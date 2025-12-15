package trackers

import (
	"ptztargettracker/utils"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

type Tracker interface {
	Calibrate(measurements []utils.PTZMeasurement) error
	CalculatePTZ(targetPose r3.Vector, cameraPose spatialmath.Pose) (ptzValues utils.PTZValues, err error)
}

type PolynomialCalibrationProvider interface {
	GetCalibration() (PolynomialCalibration, error)
}
type AbsolutePositionCalibrationProvider interface {
	GetCalibration() (CameraCalibration, error)
}
