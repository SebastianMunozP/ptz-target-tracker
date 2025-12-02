package utils

import (
	"errors"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"gonum.org/v1/gonum/mat"
)

type Ray struct {
	Direction r3.Vector
	Origin    r3.Vector
}

type RayPanTilt struct {
	Pan  float64
	Tilt float64
	Ray  Ray
}

type AbsoluteCalibration struct {
	CameraPos              r3.Vector
	PanPlane               r3.Vector
	Pan0ReferenceDirection r3.Vector // Direction in panPlane that corresponds to pan=0
}

type RayMeasurement struct {
	TargetPos r3.Vector
	Pan       float64
	Tilt      float64
}

type AbsoluteCalibrationRayMeasurements struct {
	RayId        string
	Measurements []RayMeasurement
}

type PTZCalibrationAbsolutePosition struct {
	logger                             logging.Logger
	absoluteCalibrationPanPlane        r3.Vector
	absoluteCalibrationPan0Reference   r3.Vector                                     // Default to world +X
	absoluteCalibrationRayMeasurements map[string]AbsoluteCalibrationRayMeasurements // rayId -> measurements
	panMinDeg                          float64
	panMaxDeg                          float64
	tiltMinDeg                         float64
	tiltMaxDeg                         float64
}

func (t *PTZCalibrationAbsolutePosition) addAbsoluteCalibrationMeasurement(targetPos r3.Vector, pan float64, tilt float64, rayId string) error {
	measurements, ok := t.absoluteCalibrationRayMeasurements[rayId]
	if !ok {
		measurements = AbsoluteCalibrationRayMeasurements{
			RayId:        rayId,
			Measurements: []RayMeasurement{},
		}
	}
	measurements.Measurements = append(measurements.Measurements, RayMeasurement{
		TargetPos: targetPos,
		Pan:       pan,
		Tilt:      tilt,
	})
	t.absoluteCalibrationRayMeasurements[rayId] = measurements
	return nil
}

func (t *PTZCalibrationAbsolutePosition) getAbsoluteCalibrationMeasurements(rayId string) (AbsoluteCalibrationRayMeasurements, error) {
	measurements, ok := t.absoluteCalibrationRayMeasurements[rayId]
	if !ok {
		return AbsoluteCalibrationRayMeasurements{}, errors.New("no measurements found for rayId: " + rayId)
	}
	return measurements, nil
}

func calculateAverage(values []float64) float64 {
	sum := 0.0
	for _, value := range values {
		sum += value
	}
	return sum / float64(len(values))
}
func calculateCentroid(points []r3.Vector) r3.Vector {

	XValues := make([]float64, 0, len(points))
	YValues := make([]float64, 0, len(points))
	ZValues := make([]float64, 0, len(points))

	for _, p := range points {
		XValues = append(XValues, p.X)
		YValues = append(YValues, p.Y)
		ZValues = append(ZValues, p.Z)
	}
	centroid := r3.Vector{
		X: calculateAverage(XValues),
		Y: calculateAverage(YValues),
		Z: calculateAverage(ZValues),
	}
	return centroid
}
func calculateRayFromMeasurements(measurements AbsoluteCalibrationRayMeasurements) (rayPanTilt RayPanTilt, err error) {
	rayMeasurements := measurements.Measurements
	targetPositions := make([]r3.Vector, len(rayMeasurements))
	for i, measurement := range rayMeasurements {
		targetPositions[i] = measurement.TargetPos
	}
	// The Pan and Tilt values of all the measurements should be the same as the camera should be pointing at the target positions with the same pan and tilt values
	// We allow for a small error in the Pan and Tilt values as the camera may return slightly different values due to noise or other factors
	// We will use the average of the Pan and Tilt values as the camera position
	panValues := make([]float64, len(rayMeasurements))
	tiltValues := make([]float64, len(rayMeasurements))
	for i, measurement := range rayMeasurements {
		panValues[i] = measurement.Pan
		tiltValues[i] = measurement.Tilt
	}
	averagePan := calculateAverage(panValues)
	averageTilt := calculateAverage(tiltValues)
	// No tilt/pan value should be more than 0.001 from the average
	for _, measurement := range rayMeasurements {
		if math.Abs(measurement.Pan-averagePan) > 0.001 {
			err = errors.New("pan value is not within 0.001 of the average")
			return RayPanTilt{}, err
		}
		if math.Abs(measurement.Tilt-averageTilt) > 0.001 {
			err = errors.New("tilt value is not within 0.001 of the average")
			return RayPanTilt{}, err
		}
	}
	ray, _ := FitLine3D(targetPositions)
	return RayPanTilt{Pan: averagePan, Tilt: averageTilt, Ray: ray}, nil
}

func (t *PTZCalibrationAbsolutePosition) predictPanTiltAbsolute(pos r3.Vector, cameraPos r3.Vector, panPlane r3.Vector, panZeroDirection r3.Vector) (pan, tilt float64) {
	// Normalize panPlane normal (this is in camera/panPlane coordinate frame)
	panPlaneLen := math.Sqrt(panPlane.X*panPlane.X + panPlane.Y*panPlane.Y + panPlane.Z*panPlane.Z)
	if panPlaneLen < 1e-10 {
		t.logger.Errorf("panPlane normal is zero vector")
		return 0, 0
	}
	panPlaneNormal := r3.Vector{
		X: panPlane.X / panPlaneLen,
		Y: panPlane.Y / panPlaneLen,
		Z: panPlane.Z / panPlaneLen,
	}

	// Calculate direction from camera to target (in world coordinates)
	directionWorld := pos.Sub(cameraPos)
	dirLen := math.Sqrt(directionWorld.X*directionWorld.X + directionWorld.Y*directionWorld.Y + directionWorld.Z*directionWorld.Z)
	if dirLen < 1e-10 {
		// Target is at camera position, return zero pan/tilt
		return 0, 0
	}
	directionWorld = r3.Vector{X: directionWorld.X / dirLen, Y: directionWorld.Y / dirLen, Z: directionWorld.Z / dirLen}

	// Transform direction from world coordinates to panPlane coordinate system
	// Establish orthonormal basis: panPlaneNormal = Z-axis, pan0Reference = X-axis
	// pan0Reference is already projected onto panPlane and normalized, so use it directly as panPlaneX
	panPlaneX := panZeroDirection

	// Ensure panPlaneX is normalized (should already be, but check anyway)
	panPlaneXLen := math.Sqrt(panPlaneX.X*panPlaneX.X + panPlaneX.Y*panPlaneX.Y + panPlaneX.Z*panPlaneX.Z)
	if panPlaneXLen < 1e-10 {
		// Fallback: use world +X projected onto panPlane
		worldX := r3.Vector{X: 1, Y: 0, Z: 0}
		worldXProjLen := worldX.Dot(panPlaneNormal)
		panPlaneX = r3.Vector{
			X: worldX.X - panPlaneNormal.X*worldXProjLen,
			Y: worldX.Y - panPlaneNormal.Y*worldXProjLen,
			Z: worldX.Z - panPlaneNormal.Z*worldXProjLen,
		}
		panPlaneXLen = math.Sqrt(panPlaneX.X*panPlaneX.X + panPlaneX.Y*panPlaneX.Y + panPlaneX.Z*panPlaneX.Z)
		if panPlaneXLen > 1e-10 {
			panPlaneX = r3.Vector{
				X: panPlaneX.X / panPlaneXLen,
				Y: panPlaneX.Y / panPlaneXLen,
				Z: panPlaneX.Z / panPlaneXLen,
			}
		} else {
			panPlaneX = r3.Vector{X: 1, Y: 0, Z: 0}
		}
	} else if math.Abs(panPlaneXLen-1.0) > 1e-6 {
		// Normalize if not already normalized
		panPlaneX = r3.Vector{
			X: panPlaneX.X / panPlaneXLen,
			Y: panPlaneX.Y / panPlaneXLen,
			Z: panPlaneX.Z / panPlaneXLen,
		}
	}

	// Y-axis = Z cross X (right-handed coordinate system)
	panPlaneY := panPlaneNormal.Cross(panPlaneX)
	panPlaneYLen := math.Sqrt(panPlaneY.X*panPlaneY.X + panPlaneY.Y*panPlaneY.Y + panPlaneY.Z*panPlaneY.Z)
	if panPlaneYLen > 1e-10 {
		panPlaneY = r3.Vector{
			X: panPlaneY.X / panPlaneYLen,
			Y: panPlaneY.Y / panPlaneYLen,
			Z: panPlaneY.Z / panPlaneYLen,
		}
	}

	// Transform direction from world coordinates to panPlane coordinates
	// direction_panPlane = [directionWorld · panPlaneX, directionWorld · panPlaneY, directionWorld · panPlaneNormal]
	directionPanPlane := r3.Vector{
		X: directionWorld.Dot(panPlaneX),
		Y: directionWorld.Dot(panPlaneY),
		Z: directionWorld.Dot(panPlaneNormal),
	}

	// Now both direction and panPlaneNormal are in the same coordinate system (panPlane coordinates)
	// panPlaneNormal in panPlane coords = (0, 0, 1)
	// Calculate tilt: angle between direction and panPlane
	// In panPlane coords, panPlaneNormal = (0, 0, 1), so:
	// cos(angle_from_normal) = directionPanPlane.Z
	// angle_from_normal = acos(directionPanPlane.Z)
	// angle_from_plane = π/2 - angle_from_normal
	dotWithNormal := directionPanPlane.Z
	dotWithNormal = math.Max(-1.0, math.Min(1.0, dotWithNormal)) // Clamp to avoid NaN from acos
	tiltFromNormalRad := math.Acos(dotWithNormal)
	// Convert from angle from normal to angle from plane
	tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
	// Tilt is the angle from the panPlane, where tilt=-1 corresponds to tiltMinDeg (in plane) and tilt=+1 corresponds to tiltMaxDeg
	tiltRangeDeg := t.tiltMaxDeg - t.tiltMinDeg
	if tiltRangeDeg > 0 {
		// Convert tiltFromPlaneRad (in radians) to degrees, then normalize to [-1, 1]
		// Linear mapping: tiltMinDeg → -1, tiltMaxDeg → +1
		tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
		tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
		tilt = math.Max(-1.0, math.Min(1.0, tilt))
	} else {
		tilt = 0
	}
	// In panPlane coordinates, directionInPlane is just the XY components
	// directionInPlane = (directionPanPlane.X, directionPanPlane.Y, 0)
	planeDirLen := math.Sqrt(directionPanPlane.X*directionPanPlane.X + directionPanPlane.Y*directionPanPlane.Y)
	if planeDirLen < 1e-10 {
		// Direction is parallel to panPlane normal (straight up/down), pan is undefined
		// Return pan=0, tilt calculated from angle from plane
		pan = 0
		// Convert tilt to normalized [-1, 1]
		tiltRangeDeg := t.tiltMaxDeg - t.tiltMinDeg
		if tiltRangeDeg > 0 {
			tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
			tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
			tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
			tilt = math.Max(-1.0, math.Min(1.0, tilt))
		} else {
			tilt = 0
		}
		return pan, tilt
	}

	// In panPlane coordinates, pan=0 reference is along the +X axis (panPlaneX direction)
	// pan0Ref in panPlane coords = (1, 0, 0)
	// Calculate pan angle using atan2 for full 360° range
	// pan = atan2(directionPanPlane.Y, directionPanPlane.X)
	panRad := math.Atan2(directionPanPlane.Y, directionPanPlane.X)
	panDeg := panRad * 180.0 / math.Pi

	// Normalize panDeg to [0, 360) range
	// Handle negative angles by adding 360
	if panDeg < 0 {
		panDeg += 360
	}
	// Handle angles >= 360 by subtracting 360
	if panDeg >= 360 {
		panDeg -= 360
	}

	// Convert pan from degrees to normalized [-1, 1] range
	panRangeDeg := t.panMaxDeg - t.panMinDeg
	if panRangeDeg > 0 {
		// Check if this is a wrap-around range (>= 355 degrees, starting at 0)
		// This handles both full 360° ranges and ranges close to 360° (like [0, 355])
		if panRangeDeg >= 355 && t.panMinDeg == 0 {
			// Wrap-around mapping for full/near-full circle: [0, halfRange] → [0, 1], [halfRange, maxRange] → [1, -1, 0]
			halfRange := panRangeDeg / 2.0
			if panDeg <= halfRange {
				pan = panDeg / halfRange
			} else {
				pan = panDeg/halfRange - 2.0
			}
			pan = math.Max(-1.0, math.Min(1.0, pan))
		} else {
			// Standard mapping: [panMinDeg, panMaxDeg] → [-1, 1]
			// Maps panMinDeg → -1, (panMinDeg+panMaxDeg)/2 → 0, panMaxDeg → +1
			pan = (panDeg-t.panMinDeg)/panRangeDeg*2.0 - 1.0
			pan = math.Max(-1.0, math.Min(1.0, pan))
		}
	} else {
		pan = 0
	}

	// Recalculate tilt using angle from plane (already calculated above, but recalculate for consistency)
	tiltRangeDeg = t.tiltMaxDeg - t.tiltMinDeg
	if tiltRangeDeg > 0 {
		tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
		tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
		tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
		tilt = math.Max(-1.0, math.Min(1.0, tilt))
	} else {
		tilt = 0
	}
	return pan, tilt
}

/*
This function is used to get the camera position and pan plane from the two rays
The camera position is the point where the two rays intersect
The pan plane is the plane that is perpendicular to the two rays and contains the camera position at its origin such that the pan value of 0 is aligned with the +X axis of the pan plane
*/

func (t *PTZCalibrationAbsolutePosition) calculateCameraPositionAndPanPlane(rayPanTilt1 RayPanTilt, rayPanTilt2 RayPanTilt) (cameraPos r3.Vector, panPlane r3.Vector, panZeroDirection r3.Vector, tiltZeroDirection r3.Vector) {
	ray1 := rayPanTilt1.Ray
	ray2 := rayPanTilt2.Ray
	// Calculate the closest points on the two rays (not assuming intersection)
	// Ray: P = O + t*D, minimize |(O1 + t1*D1) - (O2 + t2*D2)|
	// Analytical solution for t1 and t2:
	d1 := ray1.Direction
	d2 := ray2.Direction
	o1 := ray1.Origin
	o2 := ray2.Origin

	// w0 is the vector from the origin of ray2 to the origin of ray1
	w0 := o1.Sub(o2)
	a := d1.Dot(d1)
	b := d1.Dot(d2)
	c := d2.Dot(d2)
	d := d1.Dot(w0)
	e := d2.Dot(w0)

	denom := a*c - b*b
	var t1, t2 float64
	if math.Abs(denom) > 1e-8 {
		t1 = (b*e - c*d) / denom
		t2 = (a*e - b*d) / denom
	} else {
		t1 = 0
		t2 = 0
	}

	// pt1 and pt2 are the closest points on the two rays
	pt1 := o1.Add(d1.Mul(t1))
	pt2 := o2.Add(d2.Mul(t2))
	// Use the midpoint between the two closest points as the camera position estimate
	cameraPos = r3.Vector{
		X: (pt1.X + pt2.X) / 2,
		Y: (pt1.Y + pt2.Y) / 2,
		Z: (pt1.Z + pt2.Z) / 2,
	}
	// The pan plane is always horizontal (normal = (0, 0, 1))
	// This simplifies calculations and makes tilt independent of the measurement angles
	panPlane = r3.Vector{X: 0, Y: 0, Z: 1}

	// The zero pan reference is panPlane projection ofthe vector that points from the camera position to the point where the camera is pointing at with pan=0
	panZeroDirection = r3.Vector{X: 1, Y: 0, Z: 0}

	// Zero tilt reference is the vector that points from the camera position to the point where the camera is pointing at with tilt=0, this depends on the minDeg and maxDeg values.
	// We assume that a pan value of 0 is the midpoint of the pan range, so we can calculate the zero tilt reference as the vector that points from the camera position to the point where the camera is pointing at with pan=0
	tiltZeroDeg := (t.tiltMaxDeg + t.tiltMinDeg) / 2.0
	// Convert tiltZeroDeg to radians
	tiltZeroRad := tiltZeroDeg * math.Pi / 180.0
	// Now orient this vector to point to the zero tilt reference
	tiltZeroDirection = r3.Vector{
		X: math.Cos(tiltZeroRad),
		Y: 0,
		Z: math.Sin(tiltZeroRad),
	}
	// Normalize the tilt zero reference
	tiltZeroDirection = tiltZeroDirection.Normalize()
	return cameraPos, panPlane, panZeroDirection, tiltZeroDirection
}

// FitLine3D fits a 3D line to a set of points using Principal Component Analysis (PCA).
// Returns the best-fit line as a Ray (origin point and direction vector) and the residual error.
func FitLine3D(points []r3.Vector) (Ray, float64) {
	if len(points) < 2 {
		return Ray{}, 0
	}

	// Step 1: Calculate centroid (mean point)
	xValues := make([]float64, len(points))
	yValues := make([]float64, len(points))
	zValues := make([]float64, len(points))
	for i, p := range points {
		xValues[i] = p.X
		yValues[i] = p.Y
		zValues[i] = p.Z
	}

	centroid := calculateCentroid(points)

	// Step 2: Build mean-centered data matrix (n x 3)
	n := len(points)
	data := mat.NewDense(n, 3, nil)
	for i, p := range points {
		data.Set(i, 0, p.X-centroid.X)
		data.Set(i, 1, p.Y-centroid.Y)
		data.Set(i, 2, p.Z-centroid.Z)
	}

	// Step 3: Compute covariance matrix manually
	// Covariance = (1/(n-1)) * X^T * X where X is the mean-centered data matrix (n x 3)
	cov := mat.NewSymDense(3, nil)
	scale := 1.0 / float64(n-1)
	for i := 0; i < 3; i++ {
		for j := i; j < 3; j++ {
			var sum float64
			for k := 0; k < n; k++ {
				sum += data.At(k, i) * data.At(k, j)
			}
			cov.SetSym(i, j, scale*sum)
		}
	}

	// Step 4: Eigen decomposition (symmetric matrix -> use EigenSym)
	var eig mat.EigenSym
	if !eig.Factorize(cov, true) {
		return Ray{}, math.NaN() // Failed to factorize
	}

	// Get eigenvalues and eigenvectors
	eigenvals := eig.Values(nil)
	eigenvectors := mat.NewDense(3, 3, nil)
	eig.VectorsTo(eigenvectors)

	// Find index of largest eigenvalue (principal component)
	maxIdx := 0
	for i := 1; i < 3; i++ {
		if eigenvals[i] > eigenvals[maxIdx] {
			maxIdx = i
		}
	}

	// Extract direction vector (principal eigenvector)
	direction := r3.Vector{
		X: eigenvectors.At(0, maxIdx),
		Y: eigenvectors.At(1, maxIdx),
		Z: eigenvectors.At(2, maxIdx),
	}

	// Normalize direction
	dirLen := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)
	if dirLen > 1e-10 {
		direction.X /= dirLen
		direction.Y /= dirLen
		direction.Z /= dirLen
	}

	// Ensure direction points from centroid toward the points (not away from them)
	// Use the point furthest from the origin to determine correct direction
	// (assuming camera is near origin and targets are further away)
	if len(points) > 0 {
		maxDistFromOrigin := 0.0
		var referencePoint r3.Vector
		for _, p := range points {
			distFromOrigin := math.Sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z)
			if distFromOrigin > maxDistFromOrigin {
				maxDistFromOrigin = distFromOrigin
				referencePoint = p
			}
		}
		// Compute vector from centroid to reference point
		toReference := r3.Vector{
			X: referencePoint.X - centroid.X,
			Y: referencePoint.Y - centroid.Y,
			Z: referencePoint.Z - centroid.Z,
		}
		// Normalize toReference vector
		toRefLen := math.Sqrt(toReference.X*toReference.X + toReference.Y*toReference.Y + toReference.Z*toReference.Z)
		if toRefLen > 1e-10 {
			toReference = r3.Vector{
				X: toReference.X / toRefLen,
				Y: toReference.Y / toRefLen,
				Z: toReference.Z / toRefLen,
			}
			// If direction doesn't align with vector from centroid to reference point, flip it
			if direction.Dot(toReference) < 0 {
				direction = r3.Vector{X: -direction.X, Y: -direction.Y, Z: -direction.Z}
			}
		}
	}

	// Step 5: Calculate residual error
	var totalError float64
	for _, p := range points {
		// Vector from centroid to point
		toPoint := r3.Vector{X: p.X - centroid.X, Y: p.Y - centroid.Y, Z: p.Z - centroid.Z}

		// Project onto direction
		projLen := toPoint.Dot(direction)
		projection := r3.Vector{
			X: direction.X * projLen,
			Y: direction.Y * projLen,
			Z: direction.Z * projLen,
		}

		// Perpendicular component (error)
		perpendicular := r3.Vector{
			X: toPoint.X - projection.X,
			Y: toPoint.Y - projection.Y,
			Z: toPoint.Z - projection.Z,
		}

		totalError += perpendicular.Norm2()
	}

	residual := math.Sqrt(totalError / float64(n))

	return Ray{
		Origin:    centroid,
		Direction: direction,
	}, residual
}
