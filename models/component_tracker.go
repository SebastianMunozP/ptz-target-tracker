package models

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"ptztargettracker/trackers"
	"ptztargettracker/utils"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
	rdk_utils "go.viam.com/utils"
)

var (
	ModelComponentTracker = resource.NewModel("viam", "ptz-target-tracker", "component-tracker")
	errUnimplemented      = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(genericservice.API, ModelComponentTracker,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newComponentTracker,
		},
	)
}

type Config struct {
	TargetComponentName string   `json:"target_component_name"`
	PTZCameraName       string   `json:"ptz_camera_name"`
	OnvifPTZClientName  string   `json:"onvif_ptz_client_name"`
	UpdateRateHz        float64  `json:"update_rate_hz"`
	EnableOnStart       bool     `json:"enable_on_start"`
	PanMinDeg           float64  `json:"pan_min_deg"`
	PanMaxDeg           float64  `json:"pan_max_deg"`
	TiltMinDeg          float64  `json:"tilt_min_deg"`
	TiltMaxDeg          float64  `json:"tilt_max_deg"`
	MinZoomDistanceMM   float64  `json:"min_zoom_distance_mm"`
	MaxZoomDistanceMM   float64  `json:"max_zoom_distance_mm"`
	Deadzone            *float64 `json:"deadzone,omitempty"`

	TrackingMethod                    string                                `json:"tracking_method"` // "absolute_position" or "polynomial"
	PolynomialMethodCalibration       *trackers.PolynomialCalibration       `json:"polynomial_method_calibration,omitempty"`
	AbsolutePositionMethodCalibration *trackers.AbsolutePositionCalibration `json:"absolute_position_method_calibration,omitempty"` // Only for absolute position
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	if cfg.TargetComponentName == "" {
		return nil, nil, errors.New("target_component_name is required")
	}
	if cfg.PTZCameraName == "" {
		return nil, nil, errors.New("ptz_camera_name is required")
	}
	if cfg.OnvifPTZClientName == "" {
		return nil, nil, errors.New("onvif_ptz_client_name is required")
	}
	if cfg.UpdateRateHz <= 0 {
		return nil, nil, errors.New("update_rate_hz must be greater than 0")
	}
	if cfg.PanMinDeg == 0 && cfg.PanMaxDeg == 0 {
		cfg.PanMinDeg = 0
		cfg.PanMaxDeg = 355
	}
	if cfg.TiltMinDeg == 0 && cfg.TiltMaxDeg == 0 {
		cfg.TiltMinDeg = 5
		cfg.TiltMaxDeg = 90
	}
	if cfg.PanMaxDeg <= cfg.PanMinDeg {
		return nil, nil, errors.New("pan_max_deg must be greater than pan_min_deg")
	}
	if cfg.TiltMaxDeg <= cfg.TiltMinDeg {
		return nil, nil, errors.New("tilt_max_deg must be greater than tilt_min_deg")
	}
	if cfg.Deadzone != nil && (*cfg.Deadzone < 0 || *cfg.Deadzone > 1) {
		return nil, nil, errors.New("deadzone must be greater than or equal to 0 and less than or equal to 1 (normalized range)")
	}

	// Zoom distance validation
	if cfg.MinZoomDistanceMM < 0 {
		return nil, nil, errors.New("min_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.MaxZoomDistanceMM < 0 {
		return nil, nil, errors.New("max_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.MinZoomDistanceMM > cfg.MaxZoomDistanceMM {
		return nil, nil, errors.New("max_zoom_distance_mm must be greater than or equal to min_zoom_distance_mm")
	}

	// Tracking method validation
	if cfg.TrackingMethod != "absolute-position" && cfg.TrackingMethod != "polynomial" {
		return nil, nil, errors.New("tracking_method must be either 'absolute-sposition' or 'polynomial'")
	}
	if cfg.PolynomialMethodCalibration != nil {
		if len(cfg.PolynomialMethodCalibration.PanPolyCoeffs) != 10 {
			return nil, nil, errors.New("pan_poly_coeffs must have exactly 10 coefficients")
		}
		if len(cfg.PolynomialMethodCalibration.TiltPolyCoeffs) != 10 {
			return nil, nil, errors.New("tilt_poly_coeffs must have exactly 10 coefficients")
		}
		// Check if all coefficients are valid numbers
		for _, coeff := range cfg.PolynomialMethodCalibration.PanPolyCoeffs {
			if math.IsNaN(coeff) || math.IsInf(coeff, 0) {
				return nil, nil, errors.New("pan_poly_coeffs must contain valid numbers")
			}
		}
		for _, coeff := range cfg.PolynomialMethodCalibration.TiltPolyCoeffs {
			if math.IsNaN(coeff) || math.IsInf(coeff, 0) {
				return nil, nil, errors.New("tilt_poly_coeffs must contain valid numbers")
			}
		}
	}
	return nil, nil, nil
}

const panSpeed = 1.0
const tiltSpeed = 1.0
const zoomSpeed = 1.0

type componentTracker struct {
	resource.AlwaysRebuild
	name resource.Name

	logger logging.Logger
	cfg    *Config

	// Target component related fields
	frameSystemService  framesystem.Service
	targetComponentName string

	// ONVIF PTZ client related fields
	onvifPTZClientName string
	onvifPTZClient     generic.Resource

	// Calibration related fields
	samples                           []utils.PTZMeasurement
	tracker                           trackers.Tracker
	polynomialMethodCalibration       trackers.PolynomialCalibration
	absolutePositionMethodCalibration spatialmath.Pose

	// PTZ related fields
	lastSentTZValues utils.PTZValues
	deadzone         float64
	cameraLimits     utils.CameraLimits

	// Worker related fields
	updateRateHz float64
	worker       *rdk_utils.StoppableWorkers
}

// Close implements resource.Resource.
func (s *componentTracker) Close(ctx context.Context) error {
	s.worker.Stop()
	return nil
}

func newComponentTracker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewComponentTracker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewComponentTracker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	configJSON, _ := json.MarshalIndent(conf, "", "  ")
	logger.Debugf("Creating Component with the following config:\n%s", configJSON)

	onvifPTZClientName := resource.NewName(generic.API, conf.OnvifPTZClientName)
	onvifPTZClient, err := deps.GetResource(onvifPTZClientName)
	if err != nil {
		return nil, fmt.Errorf("failed to get ONVIF PTZ client resource: %w", err)
	}

	frameSystemService, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get frame system service: %w", err)
	}

	s := &componentTracker{
		name:                name,
		logger:              logger,
		cfg:                 conf,
		frameSystemService:  frameSystemService,
		targetComponentName: conf.TargetComponentName,
		onvifPTZClient:      onvifPTZClient,
		updateRateHz:        conf.UpdateRateHz,
		cameraLimits: utils.CameraLimits{
			ZoomMinNormalized: 0.0,
			ZoomMaxNormalized: 1.0,
			ZoomMinDistanceMM: conf.MinZoomDistanceMM,
			ZoomMaxDistanceMM: conf.MaxZoomDistanceMM,
			PanMinDeg:         conf.PanMinDeg,
			PanMaxDeg:         conf.PanMaxDeg,
			TiltMinDeg:        conf.TiltMinDeg,
			TiltMaxDeg:        conf.TiltMaxDeg,
		},
		deadzone: func() float64 {
			if conf.Deadzone != nil {
				return *conf.Deadzone
			}
			return 0.0
		}(),
		worker: rdk_utils.NewBackgroundStoppableWorkers(),
	}

	// Copy calibration if provided
	s.logger.Infof("Creating component tracker with config: %+v", conf)
	s.cfg.PolynomialMethodCalibration = conf.PolynomialMethodCalibration
	s.cfg.AbsolutePositionMethodCalibration = conf.AbsolutePositionMethodCalibration
	if s.cfg.AbsolutePositionMethodCalibration != nil && s.cfg.TrackingMethod == "absolute-position" {
		s.tracker, err = trackers.NewAbsolutePositionTrackerWithCalibration(logger, s.cameraLimits, s.cfg.AbsolutePositionMethodCalibration)
		if err != nil {
			return nil, fmt.Errorf("failed to create absolute position tracker: %w", err)
		}
	} else if s.cfg.PolynomialMethodCalibration != nil && s.cfg.TrackingMethod == "polynomial" {
		s.tracker, err = trackers.NewPolynomialTrackerWithCalibration(logger, s.cameraLimits, s.cfg.PolynomialMethodCalibration)
		if err != nil {
			return nil, fmt.Errorf("failed to create polynomial tracker: %w", err)
		}
	}
	if conf.EnableOnStart {
		s.logger.Info("Starting PTZ component tracker on start")
		s.worker.Add(s.trackingLoop)
		s.logger.Info("PTZ component tracker started")
	}

	return s, nil
}

func (s *componentTracker) Name() resource.Name {
	return s.name
}

func (t *componentTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	t.logger.Debugf("DoCommand: %+v", cmd)
	if t.cfg.EnableOnStart {
		return nil, errors.New("cannot execute commands if enable_on_start is set to true")
	}
	switch cmd["command"] {
	case "get-calibration-samples":
		return map[string]interface{}{
			"calibration-samples": t.samples,
		}, nil

	case "load-calibration-samples":
		samplesRaw, ok := cmd["calibration-samples"]
		if !ok {
			return nil, fmt.Errorf("samples field is required")
		}

		samplesArray, ok := samplesRaw.([]interface{})
		if !ok {
			return nil, fmt.Errorf("samples must be an array")
		}

		var loadedSamples []utils.PTZMeasurement
		for i, sampleRaw := range samplesArray {
			sampleMap, ok := sampleRaw.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("sample %d is not a map", i)
			}

			// Parse TargetPosition
			targetPosRaw, ok := sampleMap["TargetPosition"]
			if !ok {
				return nil, fmt.Errorf("sample %d missing TargetPosition", i)
			}
			targetPosMap, ok := targetPosRaw.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPosition is not a map", i)
			}

			targetX, ok := targetPosMap["X"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPosition.X is not a float64", i)
			}
			targetY, ok := targetPosMap["Y"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPosition.Y is not a float64", i)
			}
			targetZ, ok := targetPosMap["Z"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPosition.Z is not a float64", i)
			}

			// Parse Pan
			pan, ok := sampleMap["Pan"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d Pan is not a float64", i)
			}

			// Parse Tilt
			tilt, ok := sampleMap["Tilt"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d Tilt is not a float64", i)
			}

			sample := utils.PTZMeasurement{
				Pan:  pan,
				Tilt: tilt,
				TargetPosition: r3.Vector{
					X: targetX,
					Y: targetY,
					Z: targetZ,
				},
			}
			loadedSamples = append(loadedSamples, sample)
		}

		t.samples = loadedSamples
		t.logger.Infof("Loaded %d samples", len(t.samples))

		return map[string]interface{}{
			"status":  "success",
			"samples": len(t.samples),
		}, nil

	case "push-sample":
		// Get current target position from arm
		targetPose, err := t.getPose(ctx, t.targetComponentName)
		if err != nil {
			return nil, err
		}
		targetPos := targetPose.Pose().Point()

		// Get current pan/tilt (user has manually centered)
		ptzValues, err := t.getCameraCurrentPTZStatus(ctx)
		if err != nil {
			return nil, err
		}

		sample := utils.PTZMeasurement{
			Pan:  ptzValues.Pan,
			Tilt: ptzValues.Tilt,
			TargetPosition: r3.Vector{
				X: targetPos.X,
				Y: targetPos.Y,
				Z: targetPos.Z,
			},
		}
		t.samples = append(t.samples, sample)

		t.logger.Infof("Sample %d: (%.1f, %.1f, %.1f) → pan=%.4f, tilt=%.4f",
			len(t.samples), targetPos.X, targetPos.Y, targetPos.Z, ptzValues.Pan, ptzValues.Tilt)
		lastSample := t.samples[len(t.samples)-1]
		return map[string]interface{}{
			"sample_number": len(t.samples),
			"target":        map[string]interface{}{"x": lastSample.TargetPosition.X, "y": lastSample.TargetPosition.Y, "z": lastSample.TargetPosition.Z},
			"pan":           lastSample.Pan,
			"tilt":          lastSample.Tilt,
		}, nil

	case "pop-sample":
		if len(t.samples) == 0 {
			return nil, fmt.Errorf("no samples to remove")
		}
		t.samples = t.samples[:len(t.samples)-1]
		return map[string]interface{}{"status": "removed", "index": len(t.samples)}, nil

	case "clear-calibration":
		t.tracker = nil
		t.samples = []utils.PTZMeasurement{}
		return map[string]interface{}{"status": "cleared"}, nil

	case "calibrate":

		var err error
		if t.cfg.TrackingMethod == "polynomial" {
			if len(t.samples) < trackers.PolynomialTrackerMinSamples {
				return nil, fmt.Errorf("need at least %d samples for polynomial fit, have %d", trackers.PolynomialTrackerMinSamples, len(t.samples))
			}
			t.tracker, err = trackers.NewPolynomialTracker(t.logger, t.cameraLimits)
			if err != nil {
				return nil, err
			}
		} else {
			if len(t.samples) < trackers.AbsolutePositionTrackerMinSamples {
				return nil, fmt.Errorf("need at least %d samples for absolute position fit, have %d", trackers.AbsolutePositionTrackerMinSamples, len(t.samples))
			}
			cameraPose, err := t.getPose(ctx, t.cfg.PTZCameraName)
			t.tracker, err = trackers.NewAbsolutePositionTracker(t.logger, t.cameraLimits, cameraPose.Pose())
			if err != nil {
				return nil, err
			}
		}
		if t == nil {
			return nil, fmt.Errorf("tracker is nil after calibration")
		}
		err = t.tracker.Calibrate(t.samples)
		if err != nil {
			return map[string]interface{}{
				"status": "error",
				"error":  err.Error(),
			}, nil
		}

		if provider, ok := t.tracker.(trackers.PolynomialCalibrationProvider); ok {
			polyTracker, ok := provider.(*trackers.PolynomialTracker)
			if !ok {
				return nil, fmt.Errorf("tracker is not a polynomial tracker")
			}
			polynomialCalibration, err := polyTracker.GetCalibration()
			if err != nil {
				return nil, fmt.Errorf("failed to get polynomial calibration: %w", err)
			}
			t.logger.Infof("Calibrated with polynomial coefficients: pan=%v, tilt=%v",
				polynomialCalibration.PanPolyCoeffs, polynomialCalibration.TiltPolyCoeffs)
			return map[string]interface{}{
				"status":           "success",
				"samples_used":     len(t.samples),
				"pan_poly_coeffs":  polynomialCalibration.PanPolyCoeffs,
				"tilt_poly_coeffs": polynomialCalibration.TiltPolyCoeffs,
			}, nil
		} else if provider, ok := t.tracker.(trackers.AbsolutePositionCalibrationProvider); ok {
			absolutePositionTracker, ok := provider.(*trackers.AbsolutePositionTracker)
			if !ok {
				return nil, fmt.Errorf("tracker is not an absolute position tracker")
			}
			absolutePositionCalibration, err := absolutePositionTracker.GetCalibration()
			if err != nil {
				return nil, fmt.Errorf("failed to get absolute position calibration: %w", err)
			}
			t.logger.Infof("Calibrated with camera pose: %v", absolutePositionCalibration)
			return map[string]interface{}{
				"status":       "success",
				"samples_used": len(t.samples),
				"camera_pose":  utils.PoseToMap(absolutePositionCalibration),
			}, nil
		}
		// If we get here, it means the tracker is not a polynomial or absolute position tracker
		return map[string]interface{}{"status": "error", "message": "tracker is not a polynomial or absolute position tracker"}, nil

	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

func (t *componentTracker) getPose(ctx context.Context, componentName string) (*referenceframe.PoseInFrame, error) {
	pose, err := t.frameSystemService.GetPose(ctx, componentName, "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose for component %s: %v", componentName, err)
		return nil, fmt.Errorf("Failed to get pose for component %s: %v", componentName, err)
	}
	return pose, nil
}

func (t *componentTracker) getCameraCurrentPTZStatus(ctx context.Context) (utils.PTZValues, error) {
	ptzStatusResponse, err := t.onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command": "get-status",
	})
	if err != nil {
		t.logger.Errorf("Failed to get PTZ status: %v", err)
		return utils.PTZValues{}, err
	}
	moveStatus, ok := ptzStatusResponse["move_status"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ move status is not a map")
		return utils.PTZValues{}, fmt.Errorf("PTZ move status is not a map")
	}
	movePanTilt, ok := moveStatus["pan_tilt"].(string)
	if !ok {
		t.logger.Errorf("PTZ move pan tilt is not a string")
		return utils.PTZValues{}, fmt.Errorf("PTZ move pan tilt is not a string")
	}
	if movePanTilt != "IDLE" {
		t.logger.Debugf("PTZ pan/tilt is moving (status: %s), reading position anyway", movePanTilt)
	}
	moveZoom, ok := moveStatus["zoom"].(string)
	if !ok {
		t.logger.Errorf("PTZ move zoom is not a string")
		return utils.PTZValues{}, fmt.Errorf("PTZ move zoom is not a string")
	}
	if moveZoom != "IDLE" {
		t.logger.Debugf("PTZ zoom is moving (status: %s), reading position anyway", moveZoom)
	}
	position, ok := ptzStatusResponse["position"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ status is not a map")
		return utils.PTZValues{}, fmt.Errorf("PTZ status is not a map")
	}
	zoom, ok := position["zoom"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ zoom is not a map")
		return utils.PTZValues{}, fmt.Errorf("PTZ zoom is not a map")
	}
	zoomX, ok := zoom["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ zoom x is not a float")
		return utils.PTZValues{}, fmt.Errorf("PTZ zoom x is not a float")
	}
	panTilt, ok := position["pan_tilt"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ pan tilt is not a map")
		return utils.PTZValues{}, fmt.Errorf("PTZ pan tilt is not a map")
	}
	panTiltX, ok := panTilt["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt x is not a float")
		return utils.PTZValues{}, fmt.Errorf("PTZ pan tilt x is not a float")
	}
	panTiltY, ok := panTilt["y"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt y is not a float")
		return utils.PTZValues{}, fmt.Errorf("PTZ pan tilt y is not a float")
	}
	t.logger.Debugf("PTZ status: zoom=%.1f, pan=%.1f, tilt=%.1f", zoomX, panTiltX, panTiltY)

	return utils.PTZValues{
		Pan:  panTiltX,
		Tilt: panTiltY,
		Zoom: zoomX,
	}, nil
}

func (t *componentTracker) trackingLoop(ctx context.Context) {
	t.logger.Info("Starting tracking loop")
	t.logger.Info("Update rate: %f Hz", t.cfg.UpdateRateHz)
	var updateInterval time.Duration = time.Duration(1.0 / t.cfg.UpdateRateHz * float64(time.Second))
	t.logger.Info("Update interval: %v", updateInterval)
	ticker := time.NewTicker(updateInterval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			err := t.trackTarget(ctx)
			if err != nil {
				t.logger.Errorf("Failed to track target: %v", err)
			}
		}
	}
}

func (t *componentTracker) sendAbsoluteMove(ctx context.Context, ptzValues utils.PTZValues) error {
	currentPTZValues, err := t.getCameraCurrentPTZStatus(ctx)
	if err != nil {
		return fmt.Errorf("failed to get current PTZ status: %w", err)
	}

	// Calculate deltas to from currentPTZValues to the future PTZValues, if they are within deadzone, skip move
	panDeltaNormalized := math.Abs(ptzValues.Pan - currentPTZValues.Pan)
	tiltDeltaNormalized := math.Abs(ptzValues.Tilt - currentPTZValues.Tilt)
	// Deadzone check: skip move if target is within deadzone of current position
	// Deadzone is in normalized [0, 1] range (e.g., 0.01 = 1% of the full range)
	if t.deadzone > 0 {
		if panDeltaNormalized < t.deadzone && tiltDeltaNormalized < t.deadzone {
			t.logger.Debugf("Skipping move - within deadzone: pan_delta=%.4f (deadzone=%.4f), tilt_delta=%.4f (deadzone=%.4f)", panDeltaNormalized, t.deadzone, tiltDeltaNormalized, t.deadzone)
			return nil
		}
	}

	// Comprehensive debug log with all information
	t.logger.Debugf("Sending absolute move: target pan=%.3f (current=%.3f, delta=%.4f norm, speed=%.3f), target tilt=%.3f (current=%.3f, delta=%.4f norm, speed=%.3f), zoom=%.3f, speeds: pan=%.3f tilt=%.3f zoom=%.3f",
		ptzValues.Pan, t.lastSentTZValues.Pan, panDeltaNormalized, panSpeed,
		ptzValues.Tilt, t.lastSentTZValues.Tilt, tiltDeltaNormalized, tiltSpeed,
		ptzValues.Zoom, panSpeed, tiltSpeed, zoomSpeed)
	_, err = t.onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command":       "absolute-move",
		"pan_position":  ptzValues.Pan,
		"tilt_position": ptzValues.Tilt,
		"zoom_position": ptzValues.Zoom,
		"pan_speed":     panSpeed,
		"tilt_speed":    tiltSpeed,
		"zoom_speed":    zoomSpeed,
	})
	if err != nil {
		return fmt.Errorf("failed to send absolute move: %w", err)
	}

	t.lastSentTZValues = ptzValues

	return nil
}

func (t *componentTracker) calculatePanTiltZoom(ctx context.Context, targetPos r3.Vector) (ptzValues utils.PTZValues, err error) {
	cameraPose, err := t.getPose(ctx, t.cfg.PTZCameraName)
	if err != nil {
		return utils.PTZValues{}, fmt.Errorf("failed to get camera pose: %w", err)
	}
	if t.tracker == nil {
		return utils.PTZValues{}, errors.New("not calibrated - run calibrate first")
	}
	ptzValues, err = t.tracker.CalculatePTZ(targetPos, cameraPose.Pose())
	if err != nil {
		return utils.PTZValues{}, err
	}
	return ptzValues, nil
}

func (t *componentTracker) trackTarget(ctx context.Context) error {
	if t.tracker == nil {
		return errors.New("not calibrated - run calibrate first")
	}

	targetPose, err := t.getPose(ctx, t.targetComponentName)
	if err != nil {
		return err
	}
	targetPosition := targetPose.Pose().Point()

	ptzValues, err := t.calculatePanTiltZoom(ctx, targetPosition)
	if err != nil {
		t.logger.Errorf("Failed to predict pan/tilt/zoom: %v", err)
		return err
	}
	t.logger.Debugf("Predicted pan: %.1f, tilt: %.1f, zoom: %.1f", ptzValues.Pan, ptzValues.Tilt, ptzValues.Zoom)

	return t.sendAbsoluteMove(ctx, ptzValues)
}
