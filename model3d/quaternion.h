#ifndef MODEL3D_QUATERNION_H_
#define MODEL3D_QUATERNION_H_

#include <cmath>
#include "common/common.h"
#include "common/sqlog.h"

namespace model3d {
namespace quaternion {
// format [x,y,z,w]
inline double magnitude_sqr(double* q) {
	return q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
}
inline double magnitude(double* q) {
	return sqrt(magnitude_sqr(q));
}
inline void conjugate(double* q, double* cq) {
	cq[0] = -q[0];
	cq[1] = -q[1];
	cq[2] = -q[2];
	cq[3] = q[3];
}
inline void prod_d(double* q, double d, double* retq) {
	retq[0] = q[0]*d;
	retq[1] = q[1]*d;
	retq[2] = q[2]*d;
	retq[3] = q[3]*d;
}
inline void normalize(double* q) {
	double mag_q = magnitude(q);
	prod_d(q,1.0/mag_q,q);
}
inline void inverse(double* q, double* invq) {
	conjugate(q,invq);
	double mag_q = magnitude(q);
	prod_d(invq,1.0/(mag_q*mag_q),invq);
}
inline double inner_prod_v3(double* v1, double* v2) {
	return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
}
inline void cross_prod_v3(double* v1, double* v2, double* vr) {
	vr[0] = v1[1]*v2[2] - v1[2]*v2[1];
	vr[1] = v1[2]*v2[0] - v1[0]*v2[2];
	vr[2] = v1[0]*v2[1] - v1[1]*v2[0];
}
inline void prod_unopt(double* q1, double* q2, double* qr) {
	cross_prod_v3(q1,q2,qr);
	qr[0] += q1[3]*q2[0] + q2[3]*q1[0];
	qr[1] += q1[3]*q2[1] + q2[3]*q1[1];
	qr[2] += q1[3]*q2[2] + q2[3]*q1[2];
	qr[3] = q1[3]*q2[3] - inner_prod_v3(q1,q2);
	normalize(qr);
}
inline void prod_un(double* q1, double* q2, double* qr) {
	qr[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
	qr[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
	qr[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
	qr[3] = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2];
}
inline void prod(double* q1, double* q2, double* qr) {
	prod_un(q1,q2,qr);
	normalize(qr);
}
inline void ToRotationMatrix3x3(double* q, double* m) {
	// m: row major 3x3 matrix
  // assuming q is normalized |q| = 1;
  double xx = q[0]*q[0];
  double xy = q[0]*q[1];
  double xz = q[0]*q[2];
  double xw = q[0]*q[3];
  double yy = q[1]*q[1];
  double yz = q[1]*q[2];
  double yw = q[1]*q[3];
  double zz = q[2]*q[2];
  double zw = q[2]*q[3];

  m[0]  = 1 - 2 * ( yy + zz );
  m[1]  =     2 * ( xy - zw );
  m[2]  =     2 * ( xz + yw );
  m[3]  =     2 * ( xy + zw );
  m[4]  = 1 - 2 * ( xx + zz );
  m[5]  =     2 * ( yz - xw );
  m[6]  =     2 * ( xz - yw );
  m[7]  =     2 * ( yz + xw );
  m[8]  = 1 - 2 * ( xx + yy );
}
  
inline void ToRotationMatrix4x4(double* q, double* m) {
	// m: row major 4x4 matrix
  double m1[9];
  ToRotationMatrix3x3(q,m1);	
  m[0]  = m1[0];
  m[1]  = m1[1];
  m[2]  = m1[2];
  m[4]  = m1[3];
  m[5]  = m1[4];
  m[6]  = m1[5];
  m[8]  = m1[6];
  m[9]  = m1[7];
  m[10] = m1[8];
  m[3]  = m[7] = m[11] = m[12] = m[13] = m[14] = 0;
  m[15] = 1;
}
  
inline void RotationMatrix3x3ToQuaternion(double* mat, double* q) {
	// m: row major 3x3 matrix
  
	double trace = 1 + mat[0] + mat[4] + mat[8];
  //std::cout << "[RotationMatrix3x3ToQuaternion] Trace: " << trace << std::endl;
	double S, X, Y, Z, W;
	if ( trace>RELATIVE_SMALLER_CONSTANT ) {
		S = sqrt(trace) * 2;
		X = ( mat[7] - mat[5] ) / S;
		Y = ( mat[2] - mat[6] ) / S;
		Z = ( mat[3] - mat[1] ) / S;
		W = 0.25 * S;
	} else {
		// consider as trace < 0
		if ( mat[0] > mat[4] && mat[0] > mat[8] )  {	// Column 0: 
			S  = sqrt( 1.0 + mat[0] - mat[4] - mat[8] ) * 2;
			X = 0.25 * S;
			Y = (mat[3] + mat[1] ) / S;
			Z = (mat[2] + mat[6] ) / S;
			W = (mat[7] - mat[5] ) / S;

		} else if ( mat[4] > mat[8] ) {			// Column 1: 
			S  = sqrt( 1.0 + mat[4] - mat[0] - mat[8] ) * 2;
			X = (mat[3] + mat[1] ) / S;
			Y = 0.25 * S;
			Z = (mat[7] + mat[5] ) / S;
			W = (mat[2] - mat[6] ) / S;

		} else {						// Column 2:
			S  = sqrt( 1.0 + mat[8] - mat[0] - mat[4] ) * 2;
			X = (mat[2] + mat[6] ) / S;
			Y = (mat[7] + mat[5] ) / S;
			Z = 0.25 * S;
			W = (mat[3] - mat[1] ) / S;
		}
	}
	q[0] = X;
	q[1] = Y;
	q[2] = Z;
	q[3] = W;
  normalize(q);
}
inline void RotationMatrix4x4ToQuaternion(double* mat, double* q) {
	// m: row major 4x4 matrix
	double trace = 1 + mat[0] + mat[5] + mat[10];
	double S, X, Y, Z, W;
	if ( trace>RELATIVE_SMALLER_CONSTANT ) {
		S = sqrt(trace) * 2;
		X = ( mat[9] - mat[6] ) / S;
		Y = ( mat[2] - mat[8] ) / S;
		Z = ( mat[4] - mat[1] ) / S;
		W = 0.25 * S;
	} else if ( trace<-RELATIVE_SMALLER_CONSTANT ) {
		// should not enter here, log error
		mylog( 1, "RotationMatrixToQuaternion: Negative trace in a rotation matrix: trace = %f", trace );
    X = 0;
    Y = 0;
    Z = 0;
    W = 0;
	} else {
		// consider as trace == 0
		if ( mat[0] > mat[5] && mat[0] > mat[10] )  {	// Column 0: 
			S  = sqrt( 1.0 + mat[0] - mat[5] - mat[10] ) * 2;
			X = 0.25 * S;
			Y = (mat[4] + mat[1] ) / S;
			Z = (mat[2] + mat[8] ) / S;
			W = (mat[9] - mat[6] ) / S;

		} else if ( mat[5] > mat[10] ) {			// Column 1: 
			S  = sqrt( 1.0 + mat[5] - mat[0] - mat[10] ) * 2;
			X = (mat[4] + mat[1] ) / S;
			Y = 0.25 * S;
			Z = (mat[9] + mat[6] ) / S;
			W = (mat[2] - mat[8] ) / S;

		} else {						// Column 2:
			S  = sqrt( 1.0 + mat[10] - mat[0] - mat[5] ) * 2;
			X = (mat[2] + mat[8] ) / S;
			Y = (mat[9] + mat[6] ) / S;
			Z = 0.25 * S;
			W = (mat[4] - mat[1] ) / S;
		}
	}
	q[0] = X;
	q[1] = Y;
	q[2] = Z;
	q[3] = W;
  normalize(q);
}
inline void vector3f_normalize(float* v) {
  double mag = sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
  v[0] /= mag;
  v[1] /= mag;
  v[2] /= mag;
}
inline void vector3_normalize(double* v) {
	double mag = sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
	v[0] /= mag;
	v[1] /= mag;
	v[2] /= mag;
}
inline void AxisAngleToQuaternion(double* axis, double angle, double* q) {
	vector3_normalize(axis);
    double sin_a = sin( angle / 2 );
    double cos_a = cos( angle / 2 );

    q[0] = axis[0] * sin_a;
    q[1] = axis[1] * sin_a;
    q[2] = axis[2] * sin_a;
    q[3] = cos_a;
}
inline void ToAxisAngle(double* q, double* axis, double* angle) {
	normalize( q );
    double cos_a = q[3];
    *angle = acos( cos_a ) * 2;
    double sin_a = sqrt( 1.0 - cos_a * cos_a );
    if ( fabs( sin_a ) < 0.0005 ) {
		sin_a = 1;
	}
    axis[0] = q[0] / sin_a;
    axis[1] = q[1] / sin_a;
    axis[2] = q[2] / sin_a;
}
inline void SphericalRotationToQuaternion(double lat, double lon, double angle, double* q) {
	double sin_a = sin( angle / 2 );
    double cos_a = cos( angle / 2 );
    double sin_lat  = sin( lat );
    double cos_lat  = cos( lat );
    double sin_long = sin( lon );
    double cos_long = cos( lon );

    q[0] = sin_a * cos_lat * sin_long;
    q[1] = sin_a * sin_lat;
    q[2] = sin_a * sin_lat * cos_long;
    q[3] = cos_a;
}
inline void ToSphericalRotation(double* q, double* lat, double* lon, double* angle) {
	double cos_a = q[3];
    double sin_a  = sqrt( 1.0 - cos_a * cos_a );
    *angle  = acos( cos_a ) * 2;
    if ( fabs( sin(*angle) ) < 0.0005 ) 
		sin_a = 1;
    double tx = q[0] / sin_a;
    double ty = q[1] / sin_a;
    double tz = q[2] / sin_a;

    *lat = -asin( ty );

    if ( tx * tx + tz * tz < 0.0005 )
      *lon   = 0;
    else
       *lon  = atan2( tx, tz );

    if ( *lon < 0 )
      *lon += 360.0;
}
inline void EulerToQuaternion( double ax, double ay, double az, double* q ) {
  double vx[3] = { 1, 0, 0 };
  double vy[3] = { 0, 1, 0 };
  double vz[3] = { 0, 0, 1 };
  double qx[4], qy[4], qz[4], qt[4];

  AxisAngleToQuaternion( vx, ax, qx );
  AxisAngleToQuaternion( vy, ay, qy );
  AxisAngleToQuaternion( vz, az, qz );

  prod( qx, qy, qt );
  prod( qt, qz, q );
}
// Pitch->ax, Yaw->ay, Roll->az
inline void EulerToQuaternion2( double ax, double ay, double az, double* q ) {
	const float fSinPitch(sin(ax*0.5F));
	const float fCosPitch(cos(ax*0.5F));
	const float fSinYaw(sin(ay*0.5F));
	const float fCosYaw(cos(ay*0.5F));
	const float fSinRoll(sin(az*0.5F));
	const float fCosRoll(cos(az*0.5F));
	const float fCosPitchCosYaw(fCosPitch*fCosYaw);
	const float fSinPitchSinYaw(fSinPitch*fSinYaw);

	q[0] = fSinRoll * fCosPitchCosYaw     - fCosRoll * fSinPitchSinYaw;
	q[1] = fCosRoll * fSinPitch * fCosYaw + fSinRoll * fCosPitch * fSinYaw;
	q[2] = fCosRoll * fCosPitch * fSinYaw - fSinRoll * fSinPitch * fCosYaw;
	q[3] = fCosRoll * fCosPitchCosYaw     + fSinRoll * fSinPitchSinYaw;
}
inline void RotateVector(double* q, double* v) {
	double v2[4];
	v2[0] = v[0];
	v2[1] = v[1];
	v2[2] = v[2];
	v2[3] = 0;

	double qinv[4], qt[4];
	prod_unopt(q,v2,qt);
	inverse(q,qinv);
	prod_unopt(qt,qinv,v2);
	v[0] = v2[0];
	v[1] = v2[1];
	v[2] = v2[2];
}
inline void RotateVectorBack(double* q, double* v) {
  double cq[4];
  conjugate(q, cq);
  RotateVector(cq, v);
  //vector3_normalize(v);
}
}	// namespace quaternion
}	// namespace model3d

#endif  // MODEL3D_QUATERNION_H_