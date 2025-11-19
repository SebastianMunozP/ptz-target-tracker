package ptztargettracker

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/touch"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

var (
	PoseTracker      = resource.NewModel("viam", "ptz-target-tracker", "component-tracker")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(genericservice.API, PoseTracker,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newPoseTracker,
		},
	)
}

type Config struct {
	TargetComponentName string  `json:"target_component_name"`
	PTZCameraName       string  `json:"ptz_camera_name"`
	OnvifPTZClientName  string  `json:"onvif_ptz_client_name"`
	UpdateRateHz        float64 `json:"update_rate_hz"`
	EnableOnStart       bool    `json:"enable_on_start"`
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
	return nil, nil, nil
}

type poseTracker struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	robotClient         robot.Robot
	targetComponentName string
	onvifPTZClientName  string

	baselinePan               float64
	baselineTilt              float64
	baselineZoomX             float64
	baselineCameraOrientation *spatialmath.OrientationVectorDegrees
	baselineDirection         r3.Vector // World direction at calibration

	// Fixed camera position
	cameraPosition r3.Vector // e.g., (1600, 0, -600)
}

// Close implements resource.Resource.
func (s *poseTracker) Close(ctx context.Context) error {
	panic("unimplemented")
}

// Reconfigure implements resource.Resource.
// Subtle: this method shadows the method (AlwaysRebuild).Reconfigure of poseTracker.AlwaysRebuild.
func (s *poseTracker) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	panic("unimplemented")
}

func newPoseTracker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewPoseTracker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewPoseTracker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to connect to robot: %w", err)
	}

	s := &poseTracker{
		name:                name,
		logger:              logger,
		cfg:                 conf,
		cancelCtx:           cancelCtx,
		cancelFunc:          cancelFunc,
		robotClient:         robotClient,
		targetComponentName: conf.TargetComponentName,
		onvifPTZClientName:  conf.OnvifPTZClientName,
	}

	if conf.EnableOnStart {
		go s.trackingLoop(s.cancelCtx)
		s.logger.Info("PTZ pose tracker started")
	}

	return s, nil
}

func (s *poseTracker) Name() resource.Name {
	return s.name
}

func (t *poseTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	t.logger.Infof("DoCommand: %+v", cmd)
	switch cmd["command"] {
	case "start":
		go t.trackingLoop(t.cancelCtx)
		return map[string]interface{}{"status": "running"}, nil
	case "stop":
		t.cancelFunc()
		return map[string]interface{}{"status": "stopped"}, nil
	case "get-required-camera-to-target-orientation":
		requiredCameraToTargetOrientation, err := t.getRequiredCameraToTargetOrientation(t.cancelCtx)
		if err != nil {
			return nil, fmt.Errorf("failed to get required camera to target orientation: %v", err)
		}

		requiredCameraToTargetOrientationDict := map[string]interface{}{
			"type": "ov_degrees",
			"value": map[string]interface{}{
				"th": requiredCameraToTargetOrientation.Theta,
				"x":  requiredCameraToTargetOrientation.OX,
				"y":  requiredCameraToTargetOrientation.OY,
				"z":  requiredCameraToTargetOrientation.OZ,
			},
		}

		return map[string]interface{}{"status": "required camera to target orientation", "orientation": requiredCameraToTargetOrientationDict}, nil
	case "get-target-pose-in-camera-frame":
		targetPoseInCameraFrame := t.getTargetPoseInCameraFrame(t.cancelCtx)
		if targetPoseInCameraFrame == nil {
			return nil, fmt.Errorf("failed to get target pose in camera frame")
		}
		t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame.Pose())
		targetPoseInCameraFrameDegrees := t.poseToDictDegrees(targetPoseInCameraFrame.Pose())
		return map[string]interface{}{"status": "target pose in camera frame", "target_pose_in_camera_frame": targetPoseInCameraFrameDegrees}, nil
	case "get-camera-pose":
		cameraPose := t.getCameraPose(t.cancelCtx)
		if cameraPose == nil {
			return nil, fmt.Errorf("failed to get camera pose")
		}
		return map[string]interface{}{"status": "camera pose", "camera_pose": cameraPose}, nil
	case "get-camera-current-ptz-status":
		panTiltX, panTiltY, zoomX, err := t.getCameraCurrentPTZStatus(t.cancelCtx)
		if err != nil {
			return nil, fmt.Errorf("failed to get camera current PTZ status: %v", err)
		}
		return map[string]interface{}{"pan_tilt_x": panTiltX, "pan_tilt_y": panTiltY, "zoom_x": zoomX}, nil
	case "save-baseline-camera-ptz-and-orientation":
		err := t.recordBaseline(t.cancelCtx)
		if err != nil {
			return nil, fmt.Errorf("failed to record baseline: %v", err)
		}
		return map[string]interface{}{
			"status":        "success",
			"baselinePan":   t.baselinePan,
			"baselineTilt":  t.baselineTilt,
			"baselineZoomX": t.baselineZoomX,
			"baselineDirection": map[string]interface{}{
				"x": t.baselineDirection.X,
				"y": t.baselineDirection.Y,
				"z": t.baselineDirection.Z,
			}}, nil
	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

// Call this after calibration (camera pointing at target)
func (t *poseTracker) recordBaseline(ctx context.Context) error {
	// 1. Get current PTZ position
	panTiltX, panTiltY, zoomX, err := t.getCameraCurrentPTZStatus(ctx)
	if err != nil {
		return fmt.Errorf("failed to get camera current PTZ status: %v", err)
	}

	t.baselinePan = panTiltX
	t.baselineTilt = panTiltY
	t.baselineZoomX = zoomX

	// 2. Get target position in world
	targetPose := t.getTargetPose(ctx)
	targetPos := targetPose.Pose().Point()

	// 3. Calculate baseline direction
	t.baselineDirection = r3.Vector{
		X: targetPos.X - t.cameraPosition.X,
		Y: targetPos.Y - t.cameraPosition.Y,
		Z: targetPos.Z - t.cameraPosition.Z,
	}
	t.baselineDirection = t.baselineDirection.Mul(1.0 / t.baselineDirection.Norm())

	t.logger.Infof("Baseline calibration:")
	t.logger.Infof("  Pan: %f, Tilt: %f", t.baselinePan, t.baselineTilt)
	t.logger.Infof("  Direction: (%f, %f, %f)",
		t.baselineDirection.X, t.baselineDirection.Y, t.baselineDirection.Z)

	return nil
}

// poseToDictDegrees converts a spatialmath.Pose to a dictionary with orientation in degrees
func (s *poseTracker) poseToDictDegrees(pose spatialmath.Pose) map[string]interface{} {
	point := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()

	return map[string]interface{}{
		"translation": map[string]interface{}{
			"x": point.X,
			"y": point.Y,
			"z": point.Z,
		},
		"orientation": map[string]interface{}{
			"type": "ov_degrees",
			"value": map[string]interface{}{
				"th": ov.Theta,
				"x":  ov.OX,
				"y":  ov.OY,
				"z":  ov.OZ,
			},
		},
		"parent": "world",
	}
}

/*
Get PTZ status response:

{
  "move_status": {
    "pan_tilt": "IDLE",
    "zoom": "IDLE"
  },
  "utc_time": "2025-11-19T15:44:53Z",
  "position": {
    "zoom": {
      "x": 0.5,
      "space": "http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"
    },
    "pan_tilt": {
      "space": "http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace",
      "x": 0,
      "y": 0
    }
  }
}
*/
/*
The using must point the PTZ camera at the target pose before saving the starting pose.
This is needed to be able to link the PTZ angles to the orientation of the camera.
*/
func (t *poseTracker) getRequiredCameraToTargetOrientation(ctx context.Context) (*spatialmath.OrientationVectorDegrees, error) {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil, errors.New("failed to get frame system config")
	}
	targetPose := t.getTargetPose(ctx)
	if targetPose == nil {
		t.logger.Errorf("Failed to get target pose")
		return nil, errors.New("failed to get target pose")
	}
	t.logger.Infof("Starting target pose: %+v", targetPose)

	cameraPose := t.getCameraPose(ctx)
	if cameraPose == nil {
		t.logger.Errorf("Failed to get camera pose")
		return nil, errors.New("failed to get camera pose")
	}
	t.logger.Infof("Camera pose: %+v", cameraPose)

	ov, err := t.alignCameraToTarget(ctx, targetPose, cameraPose)
	if err != nil {
		t.logger.Errorf("Failed to align camera to target: %v", err)
		return nil, errors.New("failed to align camera to target")
	}
	t.logger.Infof("Required Camera orientation: %+v", ov)

	// Now, we need to set the camera pose to the required orientation
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil, fmt.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
	}
	newCameraPose := spatialmath.NewPose(cameraPose.Pose().Point(), ov)
	t.logger.Infof("New camera pose: %+v", newCameraPose)

	orientation := newCameraPose.Orientation().OrientationVectorDegrees()
	return orientation, nil
}

func (t *poseTracker) alignCameraToTarget(ctx context.Context, targetPoseInCameraFrame *referenceframe.PoseInFrame, cameraPose *referenceframe.PoseInFrame) (*spatialmath.OrientationVector, error) {
	t.logger.Infof("Aligning camera to target")
	t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)
	t.logger.Infof("Camera pose: %+v", cameraPose)
	// OK, We know we have the target component in the center of the camera frame, and we have the camera pose, now we need to calculat the transformation such that the camera's Z axis is aligned with the target's Z axis points towards the target.
	// This means that I want the target pose in camera frame to be such that its orienation is {ox:0, oy:0, oz:1}

	targetPose := t.getTargetPose(ctx)
	if targetPose == nil {
		t.logger.Errorf("Failed to get target pose")
		return nil, errors.New("failed to get target pose")
	}
	t.logger.Infof("Target pose: %+v", targetPose)

	targetPosition := targetPose.Pose().Point()
	t.logger.Infof("Target position: %+v", targetPosition)

	cameraPosition := cameraPose.Pose().Point()
	t.logger.Infof("Camera position: %+v", cameraPosition)

	// Now, we need to calculate the orientation of the camera such that its Z axis points towards the target position.
	// This means that I want the camera pose to be such that its orientation is {ox:0, oy:0, oz:1}

	direction := r3.Vector{
		X: targetPosition.X - cameraPosition.X,
		Y: targetPosition.Y - cameraPosition.Y,
		Z: targetPosition.Z - cameraPosition.Z,
	}
	direction = direction.Mul(1.0 / direction.Norm())

	t.logger.Infof("Direction to target: (%f, %f, %f)", direction.X, direction.Y, direction.Z)

	// Build a rotation matrix where +Z points at target
	// newZ = direction (this will be camera's +Z in world coords)
	newZ := direction

	// Choose newX perpendicular to newZ
	// Use world Z-axis to define the "up" direction
	worldUp := r3.Vector{X: 0, Y: 0, Z: 1}
	newX := newZ.Cross(worldUp)

	// Handle case where newZ is parallel to world up
	if newX.Norm() < 1e-6 {
		// Direction is straight up or down, choose arbitrary X
		newX = r3.Vector{X: 1, Y: 0, Z: 0}
	} else {
		newX = newX.Mul(1.0 / newX.Norm())
	}

	// newY completes right-handed coordinate system
	newY := newZ.Cross(newX)

	// Create rotation matrix with these basis vectors as COLUMNS
	// R = [newX | newY | newZ]
	mat := []float64{
		newX.X, newX.Y, newX.Z, // Row 1: newX
		newY.X, newY.Y, newY.Z, // Row 2: newY
		newZ.X, newZ.Y, newZ.Z, // Row 3: newZ
	}
	// Convert to orientation vector
	rotmat, _ := spatialmath.NewRotationMatrix(mat)
	ov := rotmat.OrientationVectorDegrees()

	t.logger.Infof("Orientation vector: axis=(%f, %f, %f), angle=%f°",
		ov.OX, ov.OY, ov.OZ, ov.Theta)

	return (*spatialmath.OrientationVector)(ov.OrientationVectorDegrees()), nil
}

func (t *poseTracker) getCameraPose(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil
	}
	cameraPose, err := t.robotClient.GetPose(ctx, cameraFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}
	t.logger.Infof("Camera pose: %+v", cameraPose)
	return cameraPose
}

func (t *poseTracker) getTargetPose(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	targetFramePart := touch.FindPart(fsc, t.targetComponentName)
	if targetFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.targetComponentName)
		return nil
	}
	targetPose, err := t.robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}

	return targetPose
}

func (t *poseTracker) getTargetPoseInCameraFrame(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	targetFramePart := touch.FindPart(fsc, t.targetComponentName)
	if targetFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.targetComponentName)
		return nil
	}
	targetPose, err := t.robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil
	}
	targetPoseInCameraFrame, err := t.robotClient.TransformPose(ctx, targetPose, cameraFramePart.FrameConfig.Name(), []*referenceframe.LinkInFrame{})
	if err != nil {
		t.logger.Errorf("Failed to transform target pose to camera frame: %v", err)
		return nil
	}
	t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)

	return targetPoseInCameraFrame
}

func (t *poseTracker) getCameraCurrentPTZStatus(ctx context.Context) (float64, float64, float64, error) {
	onvifPTZClientName := resource.NewName(generic.API, t.onvifPTZClientName)
	onvifPTZClient, err := t.robotClient.ResourceByName(onvifPTZClientName)
	if err != nil {
		t.logger.Errorf("Failed to get onvif PTZ client: %v", err)
		return 0, 0, 0, err
	}
	ptzStatusResponse, err := onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command": "get-status",
	})
	if err != nil {
		t.logger.Errorf("Failed to get PTZ status: %v", err)
		return 0, 0, 0, err
	}
	moveStatus, ok := ptzStatusResponse["move_status"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ move status is not a map")
		return 0, 0, 0, err
	}
	movePanTilt, ok := moveStatus["pan_tilt"].(string)
	if !ok {
		t.logger.Errorf("PTZ move pan tilt is not a string")
		return 0, 0, 0, err
	}
	if movePanTilt != "IDLE" {
		t.logger.Errorf("PTZ is not idle")
		return 0, 0, 0, err
	}
	moveZoom, ok := moveStatus["zoom"].(string)
	if !ok {
		t.logger.Errorf("PTZ move zoom is not a string")
		return 0, 0, 0, err
	}
	if moveZoom != "IDLE" {
		t.logger.Errorf("PTZ is not idle")
		return 0, 0, 0, err
	}
	position, ok := ptzStatusResponse["position"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ status is not a map")
		return 0, 0, 0, err
	}
	zoom, ok := position["zoom"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ zoom is not a map")
		return 0, 0, 0, err
	}
	zoomX, ok := zoom["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ zoom x is not a float")
		return 0, 0, 0, err
	}
	panTilt, ok := position["pan_tilt"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ pan tilt is not a map")
		return 0, 0, 0, err
	}
	panTiltX, ok := panTilt["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt x is not a float")
		return 0, 0, 0, err
	}
	panTiltY, ok := panTilt["y"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt y is not a float")
		return 0, 0, 0, err
	}
	t.logger.Infof("PTZ status: zoom=%.1f, pan=%.1f, tilt=%.1f", zoomX, panTiltX, panTiltY)

	return panTiltX, panTiltY, zoomX, nil
}

func (t *poseTracker) trackingLoop(ctx context.Context) {
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
			// 1. Get the target pose in the frame system
			err := t.trackTarget(ctx)
			if err != nil {
				t.logger.Errorf("Failed to track target: %v", err)
			}
		}
	}
}

// Main tracking loop
func (t *poseTracker) trackTarget(ctx context.Context) error {
	// 1. Get target position in world frame
	targetPose := t.getTargetPose(ctx)
	targetPos := targetPose.Pose().Point()

	// 2. Calculate current direction from camera to target
	currentDirection := r3.Vector{
		X: targetPos.X - t.cameraPosition.X,
		Y: targetPos.Y - t.cameraPosition.Y,
		Z: targetPos.Z - t.cameraPosition.Z,
	}

	distance := currentDirection.Norm()
	if distance < 1e-6 {
		return errors.New("target too close to camera")
	}
	currentDirection = currentDirection.Mul(1.0 / distance)

	// 3. Calculate pan/tilt to point at this direction
	pan, tilt := t.directionToPanTilt(currentDirection)
	zoom := t.baselineZoomX

	t.logger.Debugf("Tracking: distance=%f, pan=%f, tilt=%f", distance, pan, tilt)

	// 4. Send absolute move command
	return t.sendAbsoluteMove(ctx, pan, tilt, zoom)
}

// Convert world direction to PTZ pan/tilt values
func (t *poseTracker) directionToPanTilt(direction r3.Vector) (pan, tilt float64) {
	// Calculate the angular offset from baseline direction

	// Decompose into horizontal (XY plane) and vertical components
	// Horizontal angle (pan): project onto XY plane
	baselineHorizontal := r3.Vector{
		X: t.baselineDirection.X,
		Y: t.baselineDirection.Y,
		Z: 0,
	}
	currentHorizontal := r3.Vector{
		X: direction.X,
		Y: direction.Y,
		Z: 0,
	}

	// Normalize horizontal vectors
	baselineHorizNorm := baselineHorizontal.Norm()
	currentHorizNorm := currentHorizontal.Norm()

	var panOffset float64
	if baselineHorizNorm > 1e-6 && currentHorizNorm > 1e-6 {
		baselineHorizontal = baselineHorizontal.Mul(1.0 / baselineHorizNorm)
		currentHorizontal = currentHorizontal.Mul(1.0 / currentHorizNorm)

		// Calculate angle between horizontal projections
		// Use atan2 for signed angle
		crossZ := baselineHorizontal.X*currentHorizontal.Y - baselineHorizontal.Y*currentHorizontal.X
		dot := baselineHorizontal.Dot(currentHorizontal)
		panOffset = math.Atan2(crossZ, dot)
	}

	// Vertical angle (tilt): elevation difference
	baselineElevation := math.Asin(t.baselineDirection.Z)
	currentElevation := math.Asin(direction.Z)
	tiltOffset := currentElevation - baselineElevation

	// Convert offsets to normalized coordinates
	// Assuming ±180° pan range and ±90° tilt range
	panNorm := panOffset / math.Pi         // radians to [-1, 1]
	tiltNorm := tiltOffset / (math.Pi / 2) // radians to [-1, 1]

	// Subtract from baseline
	pan = t.baselinePan - panNorm
	tilt = t.baselineTilt - tiltNorm

	// Clamp to valid range
	pan = math.Max(-1.0, math.Min(1.0, pan))
	tilt = math.Max(-1.0, math.Min(1.0, tilt))

	return pan, tilt
}

func (t *poseTracker) sendAbsoluteMove(ctx context.Context, pan float64, tilt float64, zoom float64) error {
	onvifPTZClientName := resource.NewName(generic.API, t.onvifPTZClientName)
	onvifPTZClient, err := t.robotClient.ResourceByName(onvifPTZClientName)
	if err != nil {
		return fmt.Errorf("failed to get onvif PTZ client: %w", err)
	}
	_, err = onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command":       "absolute-move",
		"pan_position":  pan,
		"tilt_position": tilt,
		"zoom_position": zoom,
	})
	if err != nil {
		return fmt.Errorf("failed to send absolute move: %w", err)
	}
	return nil
}
