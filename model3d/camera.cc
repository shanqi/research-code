#include "camera.h"
#include "quaternion.h"
#include "third_party/Eigen/LU"
#include "third_party/Eigen/SVD"
#include "third_party/Eigen/QR"
#include "fileio/fileio_simple.h"
#include "common/common.h"

using namespace Eigen;

namespace model3d {

namespace {
  
Matrix3d Matrix3dFlipud(const Matrix3d m) {
	Matrix3d mr = m;
	mr(0,0) = m(2,0);
	mr(0,1) = m(2,1);
	mr(0,2) = m(2,2);
	mr(2,0) = m(0,0);
	mr(2,1) = m(0,1);
	mr(2,2) = m(0,2);
	return mr;
}
  
Matrix3d Matrix3dFliplr(const Matrix3d m) {
	Matrix3d mr = m;
	mr(0,0) = m(0,2);
	mr(0,2) = m(0,0);
	mr(1,0) = m(1,2);
	mr(1,2) = m(1,0);
	mr(2,0) = m(2,2);
	mr(2,2) = m(2,0);
	return mr;
}
} // namespace
  
//#define _DEBUG_COMPUTE_PROJECTION_MATRIX_
//#define _DEBUG_SET_FROM_PROJECTION_MATRIX_
//#define _DEBUG_KR_DECOMPOSITION_
//#define _DEBUG_GETR_
//#define _DEBUG_GETRRt_
  
Camera::Camera(){
	Init();
}

Camera::~Camera(){}

void Camera::Init() {
	type = UNKNOWN;
	pos[0] = 0;
	pos[1] = 0;
	pos[2] = 0;
	focal_length[0] = 0;
	focal_length[1] = 0;
  pixel_angle_step = 0;
  im_width_360 = 0;
	quaternion[0] = 0;
	quaternion[1] = 0;
	quaternion[2] = 0;
	quaternion[3] = 0;
	radial_distortion[0] = 0;
	radial_distortion[1] = 0;
	skew = 0.0;

	im_width = 0;
	im_height = 0;
  yflip = false;
  zvdir = -1;

	isProjMatReady = false;
}

void Camera::GetProjectionMatrix(double* ret_data) {
	if (!isProjMatReady) {
		ComputeProjectionMatrix();
	}
	std::copy( proj_mat, proj_mat+12, ret_data );
}
  
void Camera::GetK(Matrix3d* matK) {
	matK->setZero();
	(*matK)(0,0) = focal_length[0];
	(*matK)(1,1) = focal_length[1];
	(*matK)(0,1) = skew;
	(*matK)(0,2) = -((im_width)/2.0);
	(*matK)(1,2) = -((im_height)/2.0);
	(*matK)(2,2) = zvdir;
}
  
void Camera::GetR(Matrix3d* matR) {
	double rot_mat_data[9];
	quaternion::ToRotationMatrix3x3( quaternion, rot_mat_data );
  if (yflip==true) {
    rot_mat_data[3] = -rot_mat_data[3];
    rot_mat_data[4] = -rot_mat_data[4];
    rot_mat_data[5] = -rot_mat_data[5];
  }
  //*matR = Map<Matrix3d>(rot_mat_data, 3, 3);
	(*matR)(0,0) = rot_mat_data[0];
	(*matR)(0,1) = rot_mat_data[1];
	(*matR)(0,2) = rot_mat_data[2];
	(*matR)(1,0) = rot_mat_data[3];
	(*matR)(1,1) = rot_mat_data[4];
	(*matR)(1,2) = rot_mat_data[5];
	(*matR)(2,0) = rot_mat_data[6];
	(*matR)(2,1) = rot_mat_data[7];
	(*matR)(2,2) = rot_mat_data[8];
  
#ifdef  _DEBUG_GETR_
  PrintMatrix(*matR, "[GetR] matR");
#endif
}
  
void Camera::GetRRt(Matrix<double,3,4>* matRRt) {
	Matrix3d matR;
#ifdef _DEBUG_GETRRt_
  std::cout << "GetR" << std::endl;
#endif
	GetR(&matR);
#ifdef _DEBUG_GETRRt_
  PrintMatrix(matR, "[GetRRT] matR");
#endif
	//matR.transposeInPlace();
	Vector3d t = Vector3d(pos[0],pos[1],pos[2]);
	Vector3d Rt = matR*t;

	(*matRRt)(0,0) = matR(0,0);
	(*matRRt)(0,1) = matR(0,1);
	(*matRRt)(0,2) = matR(0,2);
	(*matRRt)(1,0) = matR(1,0);
	(*matRRt)(1,1) = matR(1,1);
	(*matRRt)(1,2) = matR(1,2);
	(*matRRt)(2,0) = matR(2,0);
	(*matRRt)(2,1) = matR(2,1);
	(*matRRt)(2,2) = matR(2,2);
	(*matRRt)(0,3) = -Rt(0);
	(*matRRt)(1,3) = -Rt(1);
	(*matRRt)(2,3) = -Rt(2);
  
} 

void Camera::ComputeProjectionMatrix() {
	Matrix3d matK;
	Matrix<double,3,4> matRRt;
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  my_printf(1, "GetK\n");
#endif
	GetK(&matK);
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  PrintMatrix(matK, "matK");
#endif
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  my_printf(1, "GetRRt\n");
#endif
	GetRRt(&matRRt);
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  PrintMatrix(matRRt, "matRRt");
#endif
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  my_printf(1, "Computing matK * matRRt \n");
#endif
	Matrix<double,3,4> matP = matK * matRRt;
#ifdef _DEBUG_COMPUTE_PROJECTION_MATRIX_
  PrintMatrix(matP, "matP");
#endif
  //PrintMatrix(matK, "matK");
  //PrintMatrix(matRRt, "matRRt");
	proj_mat[0 ] = matP(0,0);
	proj_mat[1 ] = matP(0,1);
	proj_mat[2 ] = matP(0,2);
	proj_mat[3 ] = matP(0,3);
	proj_mat[4 ] = matP(1,0);
	proj_mat[5 ] = matP(1,1);
	proj_mat[6 ] = matP(1,2);
	proj_mat[7 ] = matP(1,3);
	proj_mat[8 ] = matP(2,0);
	proj_mat[9 ] = matP(2,1);
	proj_mat[10] = matP(2,2);
	proj_mat[11] = matP(2,3);

	this->ComputeInverseProjectionMatrix();
	this->isProjMatReady = true;
}
  
void Camera::ComputeInverseProjectionMatrix() {
	// compute inverse projection matrix
	Matrix4d pmat2;
	pmat2.setZero();
	pmat2(0,0) = proj_mat[0 ];
	pmat2(0,1) = proj_mat[1 ];
	pmat2(0,2) = proj_mat[2 ];
	pmat2(0,3) = proj_mat[3 ];
	pmat2(1,0) = proj_mat[4 ];
	pmat2(1,1) = proj_mat[5 ];
	pmat2(1,2) = proj_mat[6 ];
	pmat2(1,3) = proj_mat[7 ];
	pmat2(2,0) = proj_mat[8 ];
	pmat2(2,1) = proj_mat[9 ];
	pmat2(2,2) = proj_mat[10];
	pmat2(2,3) = proj_mat[11];
	pmat2(3,3) = 1;
	Matrix4d pmat2_inv = (Matrix4d)(pmat2.inverse());

	inv_proj_mat[0 ] = pmat2_inv(0,0);
	inv_proj_mat[1 ] = pmat2_inv(0,1);
	inv_proj_mat[2 ] = pmat2_inv(0,2);
	inv_proj_mat[3 ] = pmat2_inv(0,3);
	inv_proj_mat[4 ] = pmat2_inv(1,0);
	inv_proj_mat[5 ] = pmat2_inv(1,1);
	inv_proj_mat[6 ] = pmat2_inv(1,2);
	inv_proj_mat[7 ] = pmat2_inv(1,3);
	inv_proj_mat[8 ] = pmat2_inv(2,0);
	inv_proj_mat[9 ] = pmat2_inv(2,1);
	inv_proj_mat[10] = pmat2_inv(2,2);
	inv_proj_mat[11] = pmat2_inv(2,3);
  
}

void Camera::SetFromProjectionMatrix(double* data) {
	this->type = LINEAR;
	std::copy( data, data+12, proj_mat );
	this->ComputeInverseProjectionMatrix();
	this->isProjMatReady = true;

	Matrix3d pmat1;
	pmat1(0,0) = data[0];
	pmat1(0,1) = data[1];
	pmat1(0,2) = data[2];
	pmat1(1,0) = data[4];
	pmat1(1,1) = data[5];
	pmat1(1,2) = data[6];
	pmat1(2,0) = data[8];
	pmat1(2,1) = data[9];
	pmat1(2,2) = data[10];

	Vector3d b;		// take negative values as -Rt
	b(0) = -data[3];
	b(1) = -data[7];
	b(2) = -data[11];
	
	// camera center
	JacobiSVD<Matrix3d> svd(pmat1, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Vector3d cam_center = svd.solve(b);
	pos[0] = cam_center(0);
	pos[1] = cam_center(1);
	pos[2] = cam_center(2);
  /**
  {
    Vector3d test_b = pmat1*cam_center;
    my_printf(1, "b: [%f, %f, %f]\n", b(0),b(1),b(2));
    my_printf(1, "test_b: [%f, %f, %f]\n", test_b(0),test_b(1),test_b(2));
  }
  */
	// rotation matrix 
	Matrix3d K, R;
	DecomposeKR( pmat1, &K, &R );
  zvdir = K(2,2);
#ifdef _DEBUG_SET_FROM_PROJECTION_MATRIX_
  PrintMatrix(pmat1, "pmat1");
  PrintMatrix(K, "K");
  PrintMatrix(R, "R");
  PrintMatrix( K*R, "K*R" );
#endif
  
	focal_length[0] = K(0,0);
	focal_length[1] = K(1,1);
  pixel_angle_step = abs(atan(1/focal_length[0]));
  im_width_360 = M_PI*2 / pixel_angle_step;
	skew = K(0,1);
	im_width = round(-K(0,2)*2);	// assuming priniciple point is the center of the image
	im_height = round(-K(1,2)*2);	// assuming priniciple point is the center of the image

  if (R.determinant()<0) {
    yflip = true;
    R(1,0) = -R(1,0);
    R(1,1) = -R(1,1);
    R(1,2) = -R(1,2);
  }
  
	double Rdata[9];
	Rdata[0] = R(0,0);
	Rdata[1] = R(0,1);
	Rdata[2] = R(0,2);
	Rdata[3] = R(1,0);
	Rdata[4] = R(1,1);
	Rdata[5] = R(1,2);
	Rdata[6] = R(2,0);
	Rdata[7] = R(2,1);
	Rdata[8] = R(2,2);
	quaternion::RotationMatrix3x3ToQuaternion( Rdata, quaternion );

	radial_distortion[0] = 0;
	radial_distortion[1] = 0;
  
}

void Camera::DecomposeKR(const Matrix3d pmat, Matrix3d* K, Matrix3d* R) {
	Matrix3d pmat_ud_tr = Matrix3dFlipud(pmat).transpose();
	//FullPivHouseholderQR<Matrix3d> qr(pmat_ud_tr.rows(), pmat_ud_tr.cols());
	//qr.compute(pmat_ud_tr);
  //Matrix3d Q = qr.matrixQ();
	//Matrix3d G = qr.matrixQR().triangularView<Upper>();
  
  HouseholderQR<Matrix3d> qr(pmat_ud_tr);
  Matrix3d Q = qr.householderQ();
	Matrix3d G = qr.matrixQR().triangularView<Upper>();

#ifdef _DEBUG_KR_DECOMPOSITION_
  PrintMatrix(pmat_ud_tr, "qr_decomp: pmat");
  PrintMatrix(G, "qr_decomp: K");
  PrintMatrix(Q, "qr_decomp: R");
#endif
  
	Matrix3d retK = Matrix3dFliplr(Matrix3dFlipud(G.transpose()));
	Matrix3d retR = Matrix3dFlipud(Q.transpose());
  
  if (retK(0,0)<0) {    // safe check, focal length must be positive
    retK = -retK;
    retR = -retR;
  }
  
  *K = retK;
  *R = retR;

}

void Camera::Set(CameraType type, double* data) {
	this->type = type;
	std::copy( data, data+3, this->pos );
	std::copy( data+3, data+5, this->focal_length );
	std::copy( data+5, data+9, this->quaternion );
	std::copy( data+9, data+11, this->radial_distortion );
	this->skew = data[11];
	this->isProjMatReady = false;
  
  pixel_angle_step = abs(atan(1/focal_length[0]));
  im_width_360 = M_PI*2/pixel_angle_step;
}
  
void Camera::SetCenter( Vector3d& cam_center ) {
  this->pos[0] = cam_center[0];
  this->pos[1] = cam_center[1];
  this->pos[2] = cam_center[2];
  this->isProjMatReady = false;
}
  
void Camera::Export(CameraType* type, double* data) {
	*type = this->type;
	std::copy( pos, pos+3, data );
	std::copy( focal_length, focal_length+2, data+3 );
	std::copy( quaternion, quaternion+4, data+5 );
	std::copy( radial_distortion, radial_distortion+2, data+9 );
	data[11] = skew;
}
  
void Camera::SetSize(int width, int height) {
	this->im_width = width;
	this->im_height = height;
	this->isProjMatReady = false;
}

bool Camera::IsInFrame(int w, int h) const {
	return (w>=0)&(w<im_width)&(h>=0)&(h<im_height);
}
  
bool Camera::PixelToCameraSpacePoint(double* pixel_loc, double d, double* ret_point) {
  if ( type == UNKNOWN ) {
    mylog( 1, "Camera::PointToCameraSpacePixel: Camera Type UNKNOWN\n" );
    return false;
  }
  
  double focal_length1 = focal_length[1] < RELATIVE_SMALL_CONSTANT ? focal_length[0] : focal_length[1];
  ret_point[2] = d/zvdir;
  ret_point[1] = (d*(pixel_loc[1]+1) + im_height/2.0*ret_point[2])/focal_length1;
  ret_point[0] = (d*(pixel_loc[0]+1) + im_width/2.0*ret_point[2] - skew*ret_point[1])/focal_length[0];
  
  return true;
}

bool Camera::PixelToPoint(double* pixel_loc, double d, double* ret_point) {
	if ( type == UNKNOWN ) {
		mylog( 1, "Camera::PointToPixel: Camera Type UNKNOWN\n" );
		return false;
	}
	if (!isProjMatReady) {
		ComputeProjectionMatrix();
	}
	double p[2];
	if ( type == LINEAR_PERSPECTIVE ) {
    double cx = (im_width+1)/2.0;
    double cy = (im_height+1)/2.0;
		p[0] = pixel_loc[0] - cx + 1;
		p[1] = pixel_loc[1] - cy + 1;
		double r2 = p[0]*p[0] + p[1]*p[1];
		r2 *= radial_distortion[0];
		r2 += 1;
		p[0] *= r2;
		p[1] *= r2;
		p[0] += cx;
		p[1] += cy;
	} else {
		p[0] = pixel_loc[0]+1;
		p[1] = pixel_loc[1]+1;
	}

	Vector3d p2( p[0]*d, p[1]*d, d );

	ret_point[0] = inv_proj_mat[0]*p2[0] + inv_proj_mat[1]*p2[1] + inv_proj_mat[2]*p2[2] + inv_proj_mat[3];
	ret_point[1] = inv_proj_mat[4]*p2[0] + inv_proj_mat[5]*p2[1] + inv_proj_mat[6]*p2[2] + inv_proj_mat[7];
	ret_point[2] = inv_proj_mat[8]*p2[0] + inv_proj_mat[9]*p2[1] + inv_proj_mat[10]*p2[2] + inv_proj_mat[11];

	return true;
}

bool Camera::PointToPixel(double* point, double* d, double* ret_pixel_loc) {
	if ( type == UNKNOWN ) {
		mylog( 1, "Camera::PointToPixel: Camera Type UNKNOWN" );
		return false;
	}
	if (!isProjMatReady) {
		ComputeProjectionMatrix();
	}
	double p[3];
	p[0] = proj_mat[0]*point[0] + proj_mat[1]*point[1] + proj_mat[2]*point[2] + proj_mat[3];
	p[1] = proj_mat[4]*point[0] + proj_mat[5]*point[1] + proj_mat[6]*point[2] + proj_mat[7];
	p[2] = proj_mat[8]*point[0] + proj_mat[9]*point[1] + proj_mat[10]*point[2] + proj_mat[11];
	*d = p[2];
	if ( !BadDepth(p[2]) ) {		
		p[0] /= p[2];
		p[1] /= p[2];
		ret_pixel_loc[0] = p[0]-1;
		ret_pixel_loc[1] = p[1]-1;
		if ( IsInFrame((ret_pixel_loc[0]),(ret_pixel_loc[1])) ) {
			if ( type == LINEAR_PERSPECTIVE ) {
				// take radial distortion into account, perform inverse undistortion
				double cx = (im_width+1)/2.0;
				double cy = (im_height+1)/2.0;
				p[0] -= cx;
				p[1] -= cy;
				DistortPointR1(p,radial_distortion[0], ret_pixel_loc);
				ret_pixel_loc[0] += cx-1;
				ret_pixel_loc[1] += cy-1;
			}
		} else {			
			return false;
		}
	} else {
		return false;
	}
	return true;
}
  
bool Camera::PointToPixelV(Vector3d point, double* d, Vector2d* ret_pixel_loc) {
	double p[3] = {point(0),point(1),point(2)};
	double retp[2];
	this->PointToPixel( p, d, retp );
	*ret_pixel_loc = Vector2d( retp[0], retp[1] );
    
  return true;
}

bool Camera::PointToPixel360(double*point, double* d, double* ret_pixel_loc) {
  
  if (!isProjMatReady) {
		ComputeProjectionMatrix();
	}
	double p[3];
	p[0] = proj_mat[0]*point[0] + proj_mat[1]*point[1] + proj_mat[2]*point[2] + proj_mat[3];
	p[1] = proj_mat[4]*point[0] + proj_mat[5]*point[1] + proj_mat[6]*point[2] + proj_mat[7];
	p[2] = proj_mat[8]*point[0] + proj_mat[9]*point[1] + proj_mat[10]*point[2] + proj_mat[11];
  
  double td = p[2];
  double focal_length1 = focal_length[1] < RELATIVE_SMALL_CONSTANT ? focal_length[0] : focal_length[1];
  double camspace_point[3];
  camspace_point[2] = td/zvdir;
  camspace_point[1] = (p[1] + im_height/2.0*camspace_point[2])/focal_length1;
  camspace_point[0] = (p[0] + im_width/2.0*camspace_point[2] - skew*camspace_point[1])/focal_length[0];
  
  double sqrd = camspace_point[0]*camspace_point[0] + camspace_point[2]*camspace_point[2];
  *d = sqrt( sqrd );
  
  ret_pixel_loc[1] = abs( p[1]/p[2] );
  
  double spangle = asin( abs(camspace_point[0])/(*d) );
  if (td>0 && camspace_point[0]<0) {
    spangle = -spangle;
  } else if (td<0 && camspace_point[0]<0) {
    spangle -= M_PI;
  } else if (td<0 && camspace_point[0]>0) {
    spangle = M_PI - spangle;
  }
  
  ret_pixel_loc[0] = spangle/pixel_angle_step + im_width_360/2;
  
  return true;
}
  
void Camera::NormalCameraSpaceToWorldSpace(double* normal_in, double* normal_out) {
  double normal_rotated[3] = { normal_in[0], normal_in[1], normal_in[2] };
  quaternion::RotateVectorBack(quaternion, normal_rotated);
  
  normal_out[0] = normal_rotated[0];
  normal_out[1] = normal_rotated[1];
  normal_out[2] = normal_rotated[2];
}
  
void Camera::NormalWorldSpaceToCameraSpace(double* normal_in, double* normal_out) {
  double normal_rotated[3] = { normal_in[0], normal_in[1], normal_in[2] };
  quaternion::RotateVector(quaternion, normal_rotated);
  
  normal_out[0] = normal_rotated[0];
  normal_out[1] = normal_rotated[1];
  normal_out[2] = normal_rotated[2];
}

void Camera::NormalCameraSpaceToWorldSpace(float* normal_in, float* normal_out) {
  double dn_in[3] = { normal_in[0], normal_in[1], normal_in[2] };
  double dn_out[3];
  NormalCameraSpaceToWorldSpace(dn_in, dn_out);
  normal_out[0] = dn_out[0];
  normal_out[1] = dn_out[1];
  normal_out[2] = dn_out[2];
}

void Camera::NormalWorldSpaceToCameraSpace(float* normal_in, float* normal_out) {
  double dn_in[3] = { normal_in[0], normal_in[1], normal_in[2] };
  double dn_out[3];
  NormalWorldSpaceToCameraSpace(dn_in, dn_out);
  normal_out[0] = dn_out[0];
  normal_out[1] = dn_out[1];
  normal_out[2] = dn_out[2];
}

void Camera::DistortPointR1(double* pt, double k1, double* pr) {
	if ( IS_ZERO(k1) ) {
		pr[0] = pt[0];
		pr[1] = pt[1];
	}
	const double t2 = pt[1]*pt[1];
	const double t3 = t2*t2*t2;
	const double t4 = pt[0]*pt[0];
	const double t7 = k1*(t2+t4);
	if (k1 > 0) {
		const double t8 = 1.0/t7;
		const double t10 = t3/(t7*t7);
		const double t14 = sqrt(t10*(0.25+t8/27.0));
		const double t15 = t2*t8*pt[1]*0.5;
		const double t17 = pow(t14+t15,1.0/3.0);
		const double t18 = t17-t2*t8/(t17*3.0);
		pr[0] = t18*pt[0]/pt[1];
		pr[1] = t18;
	} else {
		const double t9 = t3/(t7*t7*4.0);
		const double t11 = t3/(t7*t7*t7*27.0);
		const std::complex<double> t12 = t9+t11;
		const std::complex<double> t13 = sqrt(t12);
		const double t14 = t2/t7;
		const double t15 = t14*pt[1]*0.5;
		const std::complex<double> t16 = t13+t15;
		const std::complex<double> t17 = pow(t16,1.0/3.0);
		const std::complex<double> t18 = (t17+t14/
			(t17*3.0))*std::complex<double>(0.0,sqrt(3.0));
		const std::complex<double> t19 = -0.5*(t17+t18)+t14/(t17*6.0);
		pr[0] = t19.real()*pt[0]/pt[1];
		pr[1] = t19.real();
	}
}
  
void Camera::LoadFromPMVSTxt( const char* pmvs_txt_filename ) {
	std::vector<std::string> lines;
	fileio::LoadFile( pmvs_txt_filename, &lines );

	double proj_mat_data[12];
	for ( int j=1; j<4; ++j ) {
		std::vector<string> vals = StringSplit( lines[j], " " );
		proj_mat_data[(j-1)*4  ] = atof(vals[0].c_str());
		proj_mat_data[(j-1)*4+1] = atof(vals[1].c_str());
		proj_mat_data[(j-1)*4+2] = atof(vals[2].c_str());
		proj_mat_data[(j-1)*4+3] = atof(vals[3].c_str());
	}
	this->SetFromProjectionMatrix(proj_mat_data);
}
  
void Camera::GetImageSize(int* width, int* height) {
	*width = im_width;
	*height = im_height;
}
  
void Camera::SetImageSizeUnsafe(int width, int height) {
	im_width = width;
	im_height = height;
}
  
void Camera::GetCenter( Vector3d* cam_center ) {
	*cam_center = Vector3d( pos[0], pos[1], pos[2] );
}

double Camera::GetFocalLength() {
  return focal_length[0];
}
  
void Camera::PrintProjectionMatrix() {
  char txtdata[1024];
  SPrintProjectionMatrix(txtdata);
  my_printf(1, txtdata);
}

void Camera::SPrintProjectionMatrix(char* txtdata) {
  if (!(this->isProjMatReady)) {
    this->ComputeProjectionMatrix();
  }
  sprintf(txtdata, "Camera projection matrix\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",
          proj_mat[0], proj_mat[1], proj_mat[2], proj_mat[3],
          proj_mat[4], proj_mat[5], proj_mat[6], proj_mat[7],
          proj_mat[8], proj_mat[9], proj_mat[10], proj_mat[11]);
}
  
void Camera::PrintCameraParameters() {
  char txtdata[1024];
  this->SPrintCameraParameters(txtdata);
  my_printf(1, txtdata);
}

void Camera::SPrintCameraParameters(char* txtdata) {
  sprintf(txtdata, "Camera Parameters:\nPosition [%f, %f, %f]\nFocal length [%f, %f]\nQuaternion [%f, %f, %f, %f]\nDistortion [%f, %f], skew %f\nImage Size [%d, %d]\n",
          pos[0], pos[1], pos[2], focal_length[0], focal_length[1],
          quaternion[0], quaternion[1], quaternion[2], quaternion[3],
          radial_distortion[0], radial_distortion[1], skew, im_width, im_height);
}
void Camera::PrintMatrix(MatrixXd mat, const char* mat_name) {
  std::cout << "Print Matrix: " << mat_name << std::endl;
  std::cout << mat << std::endl;
}
  
void Camera::TestQuaternionRotationMatrix() {
  double m[9];
  double q[4];
  my_printf(1, "Quaternion input [%f, %f, %f, %f]\n", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  quaternion::ToRotationMatrix3x3(this->quaternion, m);
  quaternion::RotationMatrix3x3ToQuaternion(m, q);
  my_printf(1, "Quaternion output [%f, %f, %f, %f]\n", q[0], q[1], q[2], q[3]);
}

}	// namespace model3d
