package utils

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// ValidateMeasurements checks quality of measurements
func ValidateMeasurements(measurements []PTZMeasurement) {
	n := len(measurements)
	if n < 3 {
		fmt.Println("⚠️  Warning: Less than 3 measurements (minimum required)")
		return
	}

	// Compute centroid
	var cx, cy, cz float64
	for _, m := range measurements {
		cx += m.TargetPosition.X
		cy += m.TargetPosition.Y
		cz += m.TargetPosition.Z
	}
	cx /= float64(n)
	cy /= float64(n)
	cz /= float64(n)

	// Compute standard deviations
	var stdX, stdY, stdZ float64
	for _, m := range measurements {
		stdX += (m.TargetPosition.X - cx) * (m.TargetPosition.X - cx)
		stdY += (m.TargetPosition.Y - cy) * (m.TargetPosition.Y - cy)
		stdZ += (m.TargetPosition.Z - cz) * (m.TargetPosition.Z - cz)
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
		dx := m.TargetPosition.X - cx
		dy := m.TargetPosition.Y - cy
		dz := m.TargetPosition.Z - cz
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
func ValidateCalibration(measurements []PTZMeasurement, cameraPose spatialmath.Pose, cameraLimits CameraLimits, XYZToPanTilt func(targetPosition r3.Vector, camera spatialmath.Pose, cameraLimits CameraLimits) PanTiltResult) {
	fmt.Println("\nCalibration Validation:")
	fmt.Println("Point | Pan Error (°) | Tilt Error (°) | Status")
	fmt.Println("------|---------------|----------------|--------")

	var totalPanErr, totalTiltErr float64
	validCount := 0

	for i, m := range measurements {
		result := XYZToPanTilt(m.TargetPosition, cameraPose, cameraLimits)

		panOrigDeg := NormalizedToDegrees(m.Pan, cameraLimits.PanMinDeg, cameraLimits.PanMaxDeg)
		tiltOrigDeg := NormalizedToDegrees(m.Tilt, cameraLimits.TiltMinDeg, cameraLimits.TiltMaxDeg)

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
