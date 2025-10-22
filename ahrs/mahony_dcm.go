package ahrs

import (
	"math"
)

// Vec3 represents a 3D vector
type Vec3 struct {
	X, Y, Z float64
}

// Mat3 represents a 3x3 matrix (stored as [row][col])
type Mat3 [3][3]float64

// MahonyFilter implements the Mahony DCM attitude filter
type MahonyFilter struct {
	dcm    Mat3    // Direction Cosine Matrix (rotation from world to body)
	bias   Vec3    // Gyro bias estimate
	KpAcc  float64 // Proportional gain for accelerometer
	KpMag  float64 // Proportional gain for magnetometer
	Ki     float64 // Integral gain for bias correction
}

// EulerAngles represents roll, pitch, yaw in radians
type EulerAngles struct {
	Roll  float64 // Roll angle (radians)
	Pitch float64 // Pitch angle (radians)
	Yaw   float64 // Yaw/Heading angle (radians)
}

// NewMahonyFilter creates a new Mahony filter with default gains
func NewMahonyFilter() *MahonyFilter {
	return &MahonyFilter{
		dcm:    identityMat3(),
		bias:   Vec3{0, 0, 0},
		KpAcc:  2.0,  // Typical value for accel proportional gain
		KpMag:  0.5,  // Typical value for mag proportional gain
		Ki:     0.01, // Typical value for integral gain
	}
}

// Update integrates gyro, accel, and optional magnetometer measurements
// gyro: angular velocity in rad/s (body frame)
// accel: acceleration in m/s² or g's (body frame)
// mag: magnetic field vector (body frame), can be nil if unavailable
// dt: time step in seconds
func (f *MahonyFilter) Update(gyro, accel Vec3, mag *Vec3, dt float64) {
	if dt <= 0.0 || !isFinite(dt) {
		return
	}

	// 1) Compute correction vector in body frame
	errB := f.correctionBody(accel, mag)

	// 2) PI controller on gyro bias
	if f.Ki > 0.0 && errB.allFinite() {
		f.bias = f.bias.add(errB.scale(f.Ki * dt))
	}

	// 3) Corrected angular rate
	omegaB := gyro.add(errB.scale(f.KpAcc)).sub(f.bias)
	if !omegaB.allFinite() {
		return
	}

	// 4) Integrate DCM: R_dot = R * [ω]_x (forward Euler)
	wx := skewSymmetric(omegaB)
	dcmDot := f.dcm.multiply(wx)
	f.dcm = f.dcm.add(dcmDot.scale(dt))

	// 5) Keep DCM orthonormal (Gram-Schmidt on columns)
	f.orthonormalize()
}

// correctionBody computes the correction vector in body frame from accel and optional mag
func (f *MahonyFilter) correctionBody(accel Vec3, mag *Vec3) Vec3 {
	err := Vec3{0, 0, 0}

	// Gravity: world +Z up
	zWorld := Vec3{0, 0, 1}

	// Predicted gravity direction in body frame (world->body)
	gEstBody := f.dcm.transpose().multiplyVec(zWorld)

	// Measured gravity (normalize accel)
	an := accel.norm()
	if isFinite(an) && an > 0.5 {
		gMeasBody := accel.scale(1.0 / an)
		if gMeasBody.allFinite() && gEstBody.allFinite() {
			err = err.add(gMeasBody.cross(gEstBody))
		}
	}

	// Magnetometer: use horizontal component for heading
	if mag != nil {
		mn := mag.norm()
		if isFinite(mn) && mn > 1e-6 {
			mB := mag.scale(1.0 / mn)

			// Remove vertical component to isolate heading
			mVert := mB.dot(gEstBody)
			mHB := mB.sub(gEstBody.scale(mVert))
			mh := mHB.norm()

			if isFinite(mh) && mh > 1e-6 {
				mHB = mHB.scale(1.0 / mh)

				// World +X expressed in body (desired magnetic heading axis)
				exB := f.dcm.transpose().multiplyVec(Vec3{1, 0, 0})

				// Weight magnetometer relative to accel term
				w := f.KpMag / math.Max(f.KpAcc, 1e-6)
				err = err.add(mHB.cross(exB).scale(w))
			}
		}
	}

	if !err.allFinite() {
		return Vec3{0, 0, 0}
	}
	return err
}

// orthonormalize performs Gram-Schmidt orthonormalization on DCM columns
func (f *MahonyFilter) orthonormalize() {
	// Columns of R (world axes expressed in body)
	c0 := f.dcm.col(0)
	c1 := f.dcm.col(1)

	// 1) Normalize first column
	n0 := c0.norm()
	if isFinite(n0) && n0 > 0.0 {
		c0 = c0.scale(1.0 / n0)
	} else {
		c0 = Vec3{1, 0, 0}
	}

	// 2) Make second column orthogonal to first and normalize
	c1 = c1.sub(c0.scale(c0.dot(c1)))
	n1 := c1.norm()
	if isFinite(n1) && n1 > 0.0 {
		c1 = c1.scale(1.0 / n1)
	} else {
		c1 = Vec3{0, 1, 0}
	}

	// 3) Third column from cross product, normalize
	c2 := c0.cross(c1)
	n2 := c2.norm()
	if isFinite(n2) && n2 > 0.0 {
		c2 = c2.scale(1.0 / n2)
	} else {
		c2 = Vec3{0, 0, 1}
	}

	f.dcm.setCol(0, c0)
	f.dcm.setCol(1, c1)
	f.dcm.setCol(2, c2)
}

// GetEuler extracts Euler angles from DCM (Aerospace ZYX: yaw-pitch-roll)
func (f *MahonyFilter) GetEuler() EulerAngles {
	R := f.dcm
	// Clamp for safety against floating point drift
	s := clamp(R[2][0], -1.0, 1.0)
	pitch := -math.Asin(s)
	roll := math.Atan2(R[2][1], R[2][2])
	yaw := math.Atan2(R[1][0], R[0][0])
	return EulerAngles{Roll: roll, Pitch: pitch, Yaw: yaw}
}

// GetDCM returns the current Direction Cosine Matrix
func (f *MahonyFilter) GetDCM() Mat3 {
	return f.dcm
}

// GetBias returns the current gyro bias estimate
func (f *MahonyFilter) GetBias() Vec3 {
	return f.bias
}

// SetGains sets the filter gains
func (f *MahonyFilter) SetGains(kpAcc, kpMag, ki float64) {
	f.KpAcc = kpAcc
	f.KpMag = kpMag
	f.Ki = ki
}

// --- Vec3 methods ---

func (v Vec3) add(other Vec3) Vec3 {
	return Vec3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vec3) sub(other Vec3) Vec3 {
	return Vec3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vec3) scale(s float64) Vec3 {
	return Vec3{v.X * s, v.Y * s, v.Z * s}
}

func (v Vec3) dot(other Vec3) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

func (v Vec3) cross(other Vec3) Vec3 {
	return Vec3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

func (v Vec3) norm() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v Vec3) allFinite() bool {
	return isFinite(v.X) && isFinite(v.Y) && isFinite(v.Z)
}

// --- Mat3 methods ---

func identityMat3() Mat3 {
	return Mat3{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}
}

func (m Mat3) add(other Mat3) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] + other[i][j]
		}
	}
	return result
}

func (m Mat3) scale(s float64) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			result[i][j] = m[i][j] * s
		}
	}
	return result
}

func (m Mat3) multiply(other Mat3) Mat3 {
	var result Mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			sum := 0.0
			for k := 0; k < 3; k++ {
				sum += m[i][k] * other[k][j]
			}
			result[i][j] = sum
		}
	}
	return result
}

func (m Mat3) multiplyVec(v Vec3) Vec3 {
	return Vec3{
		m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
		m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
		m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
	}
}

func (m Mat3) transpose() Mat3 {
	return Mat3{
		{m[0][0], m[1][0], m[2][0]},
		{m[0][1], m[1][1], m[2][1]},
		{m[0][2], m[1][2], m[2][2]},
	}
}

// Transpose returns the transpose (exported for testing)
func (m Mat3) Transpose() Mat3 {
	return m.transpose()
}

// Multiply performs matrix multiplication (exported for testing)
func (m Mat3) Multiply(other Mat3) Mat3 {
	return m.multiply(other)
}

func (m Mat3) col(idx int) Vec3 {
	return Vec3{m[0][idx], m[1][idx], m[2][idx]}
}

func (m *Mat3) setCol(idx int, v Vec3) {
	m[0][idx] = v.X
	m[1][idx] = v.Y
	m[2][idx] = v.Z
}

// skewSymmetric creates a skew-symmetric matrix from a vector
// Used to represent cross product as matrix multiplication
func skewSymmetric(v Vec3) Mat3 {
	return Mat3{
		{0, -v.Z, v.Y},
		{v.Z, 0, -v.X},
		{-v.Y, v.X, 0},
	}
}

// --- Utility functions ---

func isFinite(x float64) bool {
	return !math.IsNaN(x) && !math.IsInf(x, 0)
}

func clamp(x, min, max float64) float64 {
	if x < min {
		return min
	}
	if x > max {
		return max
	}
	return x
}

// RadToDeg converts radians to degrees
func RadToDeg(rad float64) float64 {
	return rad * 180.0 / math.Pi
}

// DegToRad converts degrees to radians
func DegToRad(deg float64) float64 {
	return deg * math.Pi / 180.0
}