package models

import (
	"context"
	"errors"
	"image"
	"image/color"
	"image/draw"
	"math"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	AimingCamera = resource.NewModel("viam", "ptz-target-tracker", "aiming-camera")
)

func init() {
	resource.RegisterComponent(camera.API, AimingCamera,
		resource.Registration[camera.Camera, *AimingCameraConfig]{
			Constructor: newAimingCamera,
		},
	)
}

type AimingCameraConfig struct {
	resource.TriviallyValidateConfig
	CameraName      string `json:"camera_name"`
	CrosshairSize   int    `json:"crosshair_size"`   // Length of crosshair lines from center
	CrosshairThick  int    `json:"crosshair_thick"`  // Thickness of crosshair lines
	CrosshairColor  string `json:"crosshair_color"`  // Color: "red", "green", "blue", "white", "black"
	CrosshairCircle bool   `json:"crosshair_circle"` // Add a circle at the center
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *AimingCameraConfig) Validate(path string) ([]string, []string, error) {
	if cfg.CameraName == "" {
		return nil, nil, errors.New("camera_name is required")
	}
	// Set defaults
	if cfg.CrosshairSize == 0 {
		cfg.CrosshairSize = 100
	}
	if cfg.CrosshairThick == 0 {
		cfg.CrosshairThick = 20
	}
	if cfg.CrosshairColor == "" {
		cfg.CrosshairColor = "red"
	}
	return []string{cfg.CameraName}, nil, nil
}

type aimingCamera struct {
	name           resource.Name
	logger         logging.Logger
	cfg            *AimingCameraConfig
	cancelCtx      context.Context
	cancelFunc     func()
	underlyingCam  camera.Camera
	crosshairColor color.Color
}

func newAimingCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (camera.Camera, error) {
	conf, err := resource.NativeConfig[*AimingCameraConfig](rawConf)
	if err != nil {
		return nil, err
	}

	// Get underlying camera
	cam, err := camera.FromDependencies(deps, conf.CameraName)
	if err != nil {
		return nil, err
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	// Parse color
	crosshairColor := parseColor(conf.CrosshairColor)

	s := &aimingCamera{
		name:           rawConf.ResourceName(),
		logger:         logger,
		cfg:            conf,
		cancelCtx:      cancelCtx,
		cancelFunc:     cancelFunc,
		underlyingCam:  cam,
		crosshairColor: crosshairColor,
	}
	return s, nil
}

func (s *aimingCamera) Reconfigure(ctx context.Context, deps resource.Dependencies, rawConf resource.Config) error {
	conf, err := resource.NativeConfig[*AimingCameraConfig](rawConf)
	if err != nil {
		return err
	}

	// Get underlying camera if it changed
	cam, err := camera.FromDependencies(deps, conf.CameraName)
	if err != nil {
		return err
	}

	s.cfg = conf
	s.underlyingCam = cam
	s.crosshairColor = parseColor(conf.CrosshairColor)
	return nil
}

func (s *aimingCamera) Name() resource.Name {
	return s.name
}

func (s *aimingCamera) Close(context.Context) error {
	s.cancelFunc()
	return nil
}

func (s *aimingCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (s *aimingCamera) Start(ctx context.Context) error {
	return nil
}

func (s *aimingCamera) Stop(ctx context.Context) error {
	return nil
}

func (s *aimingCamera) GetImage(ctx context.Context) (image.Image, error) {
	// Get image from underlying camera using Images method
	imgs, _, err := s.underlyingCam.Images(ctx, []string{"color"}, nil)
	if err != nil {
		return nil, err
	}
	if len(imgs) == 0 {
		return nil, errors.New("no images returned from underlying camera")
	}

	// Get the actual image from the NamedImage
	img, err := imgs[0].Image(ctx)
	if err != nil {
		return nil, err
	}

	// Draw crosshair on the image
	return s.drawCrosshair(img), nil
}

// drawCrosshair draws a crosshair at the center of the image
func (s *aimingCamera) drawCrosshair(img image.Image) image.Image {
	bounds := img.Bounds()
	centerX := bounds.Dx() / 2
	centerY := bounds.Dy() / 2

	// Create a mutable copy of the image
	rgba := image.NewRGBA(bounds)
	draw.Draw(rgba, bounds, img, bounds.Min, draw.Src)

	// Draw crosshair lines
	size := s.cfg.CrosshairSize
	thick := s.cfg.CrosshairThick

	// Horizontal line
	for x := centerX - size; x <= centerX+size; x++ {
		for dy := -thick / 2; dy <= thick/2; dy++ {
			if x >= 0 && x < bounds.Dx() && centerY+dy >= 0 && centerY+dy < bounds.Dy() {
				rgba.Set(x, centerY+dy, s.crosshairColor)
			}
		}
	}

	// Vertical line
	for y := centerY - size; y <= centerY+size; y++ {
		for dx := -thick / 2; dx <= thick/2; dx++ {
			if y >= 0 && y < bounds.Dy() && centerX+dx >= 0 && centerX+dx < bounds.Dx() {
				rgba.Set(centerX+dx, y, s.crosshairColor)
			}
		}
	}

	// Optional: Draw circle at center
	if s.cfg.CrosshairCircle {
		radius := 5
		for angle := 0.0; angle < 360.0; angle += 1.0 {
			rad := angle * math.Pi / 180.0
			x := centerX + int(float64(radius)*math.Cos(rad))
			y := centerY + int(float64(radius)*math.Sin(rad))
			if x >= 0 && x < bounds.Dx() && y >= 0 && y < bounds.Dy() {
				rgba.Set(x, y, s.crosshairColor)
			}
		}
	}

	return rgba
}

// parseColor converts color string to color.Color
func parseColor(colorName string) color.Color {
	switch colorName {
	case "red":
		return color.RGBA{R: 255, G: 0, B: 0, A: 255}
	case "green":
		return color.RGBA{R: 0, G: 255, B: 0, A: 255}
	case "blue":
		return color.RGBA{R: 0, G: 0, B: 255, A: 255}
	case "white":
		return color.RGBA{R: 255, G: 255, B: 255, A: 255}
	case "black":
		return color.RGBA{R: 0, G: 0, B: 0, A: 255}
	case "yellow":
		return color.RGBA{R: 255, G: 255, B: 0, A: 255}
	case "cyan":
		return color.RGBA{R: 0, G: 255, B: 255, A: 255}
	case "magenta":
		return color.RGBA{R: 255, G: 0, B: 255, A: 255}
	default:
		return color.RGBA{R: 255, G: 0, B: 0, A: 255} // Default to red
	}
}

func (s *aimingCamera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (s *aimingCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, nil
}

func (s *aimingCamera) Images(ctx context.Context, mimeTypes []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	// Get images from underlying camera
	imgs, meta, err := s.underlyingCam.Images(ctx, mimeTypes, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	// Create new named images with crosshair overlay
	resultImgs := make([]camera.NamedImage, len(imgs))
	for i, namedImg := range imgs {
		// Get the actual image
		img, err := namedImg.Image(ctx)
		if err != nil {
			return nil, resource.ResponseMetadata{}, err
		}

		// Draw crosshair on it
		imgWithCrosshair := s.drawCrosshair(img)

		// Create new NamedImage
		resultImg, err := camera.NamedImageFromImage(imgWithCrosshair, namedImg.SourceName, namedImg.MimeType())
		if err != nil {
			return nil, resource.ResponseMetadata{}, err
		}
		resultImgs[i] = resultImg
	}

	return resultImgs, meta, nil
}

func (s *aimingCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	return nil, errors.New("next point cloud not implemented")
}

func (s *aimingCamera) Properties(ctx context.Context) (camera.Properties, error) {
	// Return properties from underlying camera
	return s.underlyingCam.Properties(ctx)
}
