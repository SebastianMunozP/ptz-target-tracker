package trackers

import (
	"fmt"
	"math"
	"ptztargettracker/utils"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

type PolynomialCalibration struct {
	PanPolyCoeffs  []float64 `json:"pan_poly_coeffs"`
	TiltPolyCoeffs []float64 `json:"tilt_poly_coeffs"`
}

func solveLinearSystem10x10(A [10][10]float64, b [10]float64) [10]float64 {
	// Convert to gonum matrices
	aData := make([]float64, 100)
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			aData[i*10+j] = A[i][j]
		}
	}

	bData := make([]float64, 10)
	copy(bData, b[:])

	aMat := mat.NewDense(10, 10, aData)
	bMat := mat.NewDense(10, 1, bData)

	// Solve using QR decomposition
	var qr mat.QR
	qr.Factorize(aMat)

	var result mat.Dense
	err := qr.SolveTo(&result, false, bMat)
	if err != nil {
		// Return zeros if singular
		return [10]float64{}
	}

	// Extract coefficients
	var coeffs [10]float64
	for i := 0; i < 10; i++ {
		coeffs[i] = result.At(i, 0)
	}

	return coeffs
}

func fitPolynomialSingle(samples []utils.PTZMeasurement, getValue func(utils.PTZMeasurement) float64) []float64 {
	// Features: [x², y², z², xy, xz, yz, x, y, z, 1]
	// Build normal equations
	var XtX [10][10]float64
	var XtY [10]float64

	for _, s := range samples {
		x, y, z := s.TargetPosition.X, s.TargetPosition.Y, s.TargetPosition.Z
		features := [10]float64{
			x * x, y * y, z * z,
			x * y, x * z, y * z,
			x, y, z, 1,
		}
		val := getValue(s)

		for i := 0; i < 10; i++ {
			XtY[i] += features[i] * val
			for j := 0; j < 10; j++ {
				XtX[i][j] += features[i] * features[j]
			}
		}
	}

	coeffs := solveLinearSystem10x10(XtX, XtY)
	// Convert array to slice
	return coeffs[:]
}

// Fit: pan = Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J
func fitPolynomial(samples []utils.PTZMeasurement) (calibration PolynomialCalibration, panError, tiltError float64, err error) {
	calibration.PanPolyCoeffs = fitPolynomialSingle(samples, func(s utils.PTZMeasurement) float64 { return s.Pan })
	calibration.TiltPolyCoeffs = fitPolynomialSingle(samples, func(s utils.PTZMeasurement) float64 { return s.Tilt })

	// Calculate errors
	var panErrSum, tiltErrSum float64
	for _, s := range samples {
		predPan, predTilt, err := calculatePanTiltPolynomial(s.TargetPosition, calibration)
		if err != nil {
			return PolynomialCalibration{}, 0, 0, fmt.Errorf("failed to predict pan/tilt for error calculation: %w", err)
		}
		panErrSum += math.Abs(predPan - s.Pan)
		tiltErrSum += math.Abs(predTilt - s.Tilt)
	}

	n := float64(len(samples))
	panError = panErrSum / n
	tiltError = tiltErrSum / n

	return calibration, panError, tiltError, nil
}

func calculatePanTiltPolynomial(pos r3.Vector, calibration PolynomialCalibration) (pan, tilt float64, err error) {
	x, y, z := pos.X, pos.Y, pos.Z
	features := [10]float64{
		x * x, y * y, z * z,
		x * y, x * z, y * z,
		x, y, z, 1,
	}

	pan, tilt = 0, 0
	if len(calibration.PanPolyCoeffs) != len(calibration.TiltPolyCoeffs) {
		return 0, 0, fmt.Errorf("Calibration polynomial coefficient length mismatch: pan=%d, tilt=%d", len(calibration.PanPolyCoeffs), len(calibration.TiltPolyCoeffs))
	}
	if len(features) != len(calibration.PanPolyCoeffs) {
		return 0, 0, fmt.Errorf("Feature length mismatch: features=%d, pan_coeffs=%d", len(features), len(calibration.PanPolyCoeffs))
	}
	for i := range features {
		pan += calibration.PanPolyCoeffs[i] * features[i]
		tilt += calibration.TiltPolyCoeffs[i] * features[i]
	}
	return pan, tilt, nil
}

type PolynomialTracker struct {
	logger       logging.Logger
	cameraLimits utils.CameraLimits
	cameraPose   utils.CameraPose
	calibration  PolynomialCalibration
}

const PolynomialTrackerMinSamples = 12

func NewPolynomialTracker(logger logging.Logger, cameraLimits utils.CameraLimits) (*PolynomialTracker, error) {
	return &PolynomialTracker{
		logger:       logger,
		cameraLimits: cameraLimits,
	}, nil
}

func (a *PolynomialTracker) Calibrate(measurements []utils.PTZMeasurement) error {
	calibration, panError, tiltError, err := fitPolynomial(measurements)
	if err != nil {
		return fmt.Errorf("failed to calibrate: %w", err)
	}
	a.logger.Info(fmt.Sprintf("Calibrated with pan error: %.2f, tilt error: %.2f", panError, tiltError))
	a.calibration = calibration
	return nil
}

func (a *PolynomialTracker) CalculatePTZ(targetPose r3.Vector, cameraPose spatialmath.Pose) (utils.PTZValues, error) {
	pan, tilt, err := calculatePanTiltPolynomial(targetPose, a.calibration)
	if err != nil {
		return utils.PTZValues{}, fmt.Errorf("failed to calculate pan/tilt: %w", err)
	}
	zoom := utils.CalculateZoom(targetPose, cameraPose.Point(), a.cameraLimits)
	ptzValues := utils.PTZValues{
		Pan:  pan,
		Tilt: tilt,
		Zoom: zoom,
	}
	ptzValues = utils.ClampPTZValues(ptzValues, a.cameraLimits)
	return ptzValues, nil
}

func (a *PolynomialTracker) GetCalibration() (PolynomialCalibration, error) {
	return a.calibration, nil
}
