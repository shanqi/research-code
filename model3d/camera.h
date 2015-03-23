#ifndef MODEL3D_CAMERA_H_
#define MODEL3D_CAMERA_H_

#include "third_party/Eigen/Core"

using namespace Eigen;
using namespace std;

namespace model3d {

enum CameraType {
	LINEAR = 1,
	LINEAR_PERSPECTIVE = 2,
  PANO_360 = 3,
	UNKNOWN,
};

class Camera {
private:
	CameraType type;
	double pos[3];
	double focal_length[2];
  double pixel_angle_step;
  int im_width_360;
	double quaternion[4];
	double radial_distortion[2];	// in most cases only radial_distortion[0] has a value
	double skew;

	bool isProjMatReady;
	double proj_mat[12];
	double inv_proj_mat[12];
	
	int im_width, im_height;
  float zvdir;    // usually -1 or 1
  bool yflip;

public:
	Camera();
	~Camera();

public:
	void Init();
	void GetProjectionMatrix(double* ret_data);
	void ComputeProjectionMatrix();
	void ComputeInverseProjectionMatrix();
	void SetFromProjectionMatrix(double* data);		// data = double[12]
	void Set(CameraType type, double* data);
	void Export(CameraType* type, double* data);
	void SetSize(int width, int height);
	bool IsInFrame(int w, int h) const;
  bool PixelToCameraSpacePoint(double* pixel_loc, double d, double* ret_point);
	bool PixelToPoint(double* pixel_loc, double d, double* ret_point);
	bool PointToPixel(double* point, double* d, double* ret_pixel_loc);
  bool PointToPixelV(Vector3d point, double* d, Vector2d* ret_pixel_loc);
  bool PointToPixel360(double*point, double* d, double* ret_pixel_loc);
  void NormalCameraSpaceToWorldSpace(double* normal_in, double* normal_out);
  void NormalWorldSpaceToCameraSpace(double* normal_in, double* normal_out);
  void NormalCameraSpaceToWorldSpace(float* normal_in, float* normal_out);
  void NormalWorldSpaceToCameraSpace(float* normal_in, float* normal_out);
	void DistortPointR1(double* pt, double k1, double* pr);
	void DecomposeKR(const Matrix3d pmat, Matrix3d* K, Matrix3d* R);
	void LoadFromPMVSTxt( const char* pmvs_txt_filename );
	void GetImageSize(int* width, int* height);
	void SetImageSizeUnsafe(int width, int height);
	void GetCenter( Vector3d* cam_center );
  void SetCenter( Vector3d& cam_center );
  double GetFocalLength();
  
  void PrintProjectionMatrix();
  void SPrintProjectionMatrix(char* txtdata);
  void PrintCameraParameters();
  void SPrintCameraParameters(char* txtdata);
  void PrintMatrix(MatrixXd mat, const char* mat_name);
  
public:
  void TestQuaternionRotationMatrix();
  
private:
	void GetK(Matrix3d* matK);
	void GetR(Matrix3d* matR);
	void GetRRt(Matrix<double,3,4>* matRRt);
};	// class Camera

}	// namespace model3d

#endif  // MODEL3D_CAMERA_H_


