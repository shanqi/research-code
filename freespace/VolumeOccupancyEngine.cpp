// Author: Qi Shan <shanqi@cs.washington.edu>

#include <iostream>
#include <fstream>
#include <string>

#include "numeric/solve_2x2_3x3.h"

#include "freespace/VolumeOccupancyEngine.h"
#include "freespace/VFS_ConfigureFile.h"
#include "freespace/VolumeOccupancyEngine.h"
#include "freespace/VolumeData.h"
#include "freespace/VolumeDataFloat.h"

namespace freespace {

CVolumeOccupancyEngine::CVolumeOccupancyEngine(void) {
}


CVolumeOccupancyEngine::~CVolumeOccupancyEngine(void) {
}

inline float p3d_dis( float* p1, float* p2 ) {

	float tval1 = p1[0]-p2[0];
	float tval2 = p1[1]-p2[1];
	float tval3 = p1[2]-p2[2];
	float ret_dis = sqrt( tval1*tval1 + tval2*tval2 + tval3*tval3 );

	return ret_dis;
}

void transform_matrix_to_camera_pos( float* transform_matrix, float* ret_cam_pos ) {

	float A[3][3];
	A[0][0] = transform_matrix[0];
	A[0][1] = transform_matrix[1];
	A[0][2] = transform_matrix[2];
	A[1][0] = transform_matrix[4];
	A[1][1] = transform_matrix[5];
	A[1][2] = transform_matrix[6];
	A[2][0] = transform_matrix[8];
	A[2][1] = transform_matrix[9];
	A[2][2] = transform_matrix[10];
	
	float b[3];
	b[0] = -transform_matrix[3];
	b[1] = -transform_matrix[7];
	b[2] = -transform_matrix[11];

  numeric::slinearf::solve_3x3( A, ret_cam_pos, b );
  
}

inline void transform_matrix_pos_product( float* transform_matrix, float* point3d, float* ret_proj3d ) {
	ret_proj3d[0] = transform_matrix[0]*point3d[0]+transform_matrix[1]*point3d[1]+transform_matrix[ 2]*point3d[2]+transform_matrix[3];
	ret_proj3d[1] = transform_matrix[4]*point3d[0]+transform_matrix[5]*point3d[1]+transform_matrix[ 6]*point3d[2]+transform_matrix[7];
	ret_proj3d[2] = transform_matrix[8]*point3d[0]+transform_matrix[9]*point3d[1]+transform_matrix[10]*point3d[2]+transform_matrix[11];
}

int CVolumeOccupancyEngine::ComputePlyVolumeOccupancy( CVolumeData* volume_data, float* pdata, long pnum ) {

	float* bbox = (float*)(volume_data->BBox());
	int* vol_res = (int*)(volume_data->VolRes());
	voxel* vdata = volume_data->vdata;

	float xdis = bbox[3]-bbox[0];
	float ydis = bbox[4]-bbox[1];
	float zdis = bbox[5]-bbox[2];

	float xunitdis = xdis/vol_res[0];
	float yunitdis = ydis/vol_res[1];
	float zunitdis = zdis/vol_res[2];

	//my_printf( 2, "xyz_units [%f,%f,%f]\n", xunitdis, yunitdis, zunitdis );
	long numel_xy = ((long)vol_res[0])*vol_res[1];

	for ( long i=0; i<pnum; ++i ) {

		float cx = pdata[i*3];
		float cy = pdata[i*3+1];
		float cz = pdata[i*3+2];

		int ix = (int)((cx-bbox[0])/xunitdis);
		int iy = (int)((cy-bbox[1])/yunitdis);
		int iz = (int)((cz-bbox[2])/zunitdis);

		if ( (ix<0)||(ix>=vol_res[0]) ||  (iy<0)||(iy>=vol_res[1]) || (iz<0)||(iz>=vol_res[2])  ) {
			my_printf(2, "Point out of bounding box [%d,%d,%d]->[%f,%f,%f]\n", ix, iy, iz, cx, cy, cz);
			continue;
		}

		++ (vdata[iz*numel_xy+iy*vol_res[0]+ix].free_vote);
		++ (vdata[iz*numel_xy+iy*vol_res[0]+ix].occp_vote);
	}
	

	return 0;
}

int CVolumeOccupancyEngine::ComputePlyVolumeOccupancy( CVolumeDataFloat* volume_data, float* pdata, long pnum ) {
  
	float* bbox = (float*)(volume_data->BBox());
	int* vol_res = (int*)(volume_data->VolRes());
	voxelf* vdata = volume_data->vdata;
  
	float xdis = bbox[3]-bbox[0];
	float ydis = bbox[4]-bbox[1];
	float zdis = bbox[5]-bbox[2];
  
	float xunitdis = xdis/vol_res[0];
	float yunitdis = ydis/vol_res[1];
	float zunitdis = zdis/vol_res[2];
  
	//my_printf( 2, "xyz_units [%f,%f,%f]\n", xunitdis, yunitdis, zunitdis );
	long numel_xy = ((long)vol_res[0])*vol_res[1];
  
	for ( long i=0; i<pnum; ++i ) {
    
		float cx = pdata[i*3];
		float cy = pdata[i*3+1];
		float cz = pdata[i*3+2];
    
		int ix = (int)((cx-bbox[0])/xunitdis);
		int iy = (int)((cy-bbox[1])/yunitdis);
		int iz = (int)((cz-bbox[2])/zunitdis);
    
		if ( (ix<0)||(ix>=vol_res[0]) ||  (iy<0)||(iy>=vol_res[1]) || (iz<0)||(iz>=vol_res[2])  ) {
			my_printf(2, "Point out of bounding box [%d,%d,%d]->[%f,%f,%f]\n", ix, iy, iz, cx, cy, cz);
			continue;
		}
    
		(vdata[iz*numel_xy+iy*vol_res[0]+ix].free_vote) += 1;
		(vdata[iz*numel_xy+iy*vol_res[0]+ix].occp_vote) += 1;
	}
	
  
	return 0;
}

//#define USING_CAM_POINT_DISTANCE

int CVolumeOccupancyEngine::ComputeVolumeOccupancy( CVolumeData* volume_data, fileio::DepthfileSimple* depthdata, float* transform_matrix, float depth_scale ) {

	float* bbox = (float*)(volume_data->BBox());
	int* vol_res = (int*)(volume_data->VolRes());
	voxel* vdata = volume_data->vdata;
	int dwidth = depthdata->dwidth;
	int dheight = depthdata->dheight;
	int dwidth_m1 = dwidth - 1;
	int dheight_m1 = dheight - 1;	

	float xdis = bbox[3]-bbox[0];
	float ydis = bbox[4]-bbox[1];
	float zdis = bbox[5]-bbox[2];

	float xunitdis = xdis/vol_res[0];
	float yunitdis = ydis/vol_res[1];
	float zunitdis = zdis/vol_res[2];

	//my_printf( 2, "xyz_units [%f,%f,%f]\n", xunitdis, yunitdis, zunitdis );

	float xhalfunitdis = xunitdis/2;
	float yhalfunitdis = yunitdis/2;
	float zhalfunitdis = zunitdis/2;

	float voxel_center[3];
	float voxel_proj[3];

	long numel_depth = ((long)dwidth)*dheight;
	float max_depth = 0;
	for ( long i=0; i<numel_depth; ++i ) {
		max_depth = MY_MAX( max_depth, depthdata->depthdata[i] );
	}
	//float depth_tol = max_depth*0.0002f;
	//float depth_tol = zunitdis;
	//float depth_tol = sqrt(xunitdis*xunitdis+yunitdis*yunitdis+zunitdis*zunitdis)/2;
	

#ifdef USING_CAM_POINT_DISTANCE
	float cam_center[3];
	transform_matrix_to_camera_pos( transform_matrix, cam_center );
	float depth_tol = sqrt(xunitdis*xunitdis+yunitdis*yunitdis+zunitdis*zunitdis)/2;
#else
	float depth_tol = zunitdis/2;
#endif

	long numel_xy = ((long)vol_res[0])*vol_res[1];
	for ( int iz=0; iz<vol_res[2]; ++iz ) {
		//my_printf( 3, "iz: %d/%d\n", iz+1, vol_res[2] );
		voxel_center[2] = bbox[2]+iz*zunitdis+zhalfunitdis;
		for ( int iy=0; iy<vol_res[1]; ++iy ) {
			voxel_center[1] = bbox[1]+iy*yunitdis+yhalfunitdis;
			for ( int ix=0; ix<vol_res[0]; ++ix ) {
				voxel_center[0] = bbox[0]+ix*xunitdis+xhalfunitdis;
				
				transform_matrix_pos_product( transform_matrix, voxel_center, voxel_proj );
				if ( voxel_proj[2]<RELATIVE_SMALL_CONSTANT ) {
					continue;
				}
				float xproj = voxel_proj[0]/voxel_proj[2]*depth_scale;
				float yproj = voxel_proj[1]/voxel_proj[2]*depth_scale;
				if ( (xproj<0)||(xproj>=dwidth_m1)||(yproj<0)||(yproj>=dheight_m1) ) {		// out of image boundary, skip
					continue;
				}
#ifdef USING_CAM_POINT_DISTANCE
				float voxel_center_depth = p3d_dis( voxel_center, cam_center );
#else
				float voxel_center_depth = voxel_proj[2];
#endif

				// simple closest point approach
				bool free_flag = true;
				float depth0 = 0;
				for ( int dy=0; dy<2; ++ dy ) {
					for ( int dx=0; dx<2; ++ dx ) {
						//float tdepth1 = (depthdata->depthdata)[ (int)(xproj)+dx + ((int)(yproj)+dy)*dwidth ];
						float tdepth1 = (depthdata->depthdata)[ ((int)(xproj))+dx + (dheight_m1-((int)(yproj))+dy)*dwidth ];
						if ( dy==0 && dx==0 ) {
							depth0 = tdepth1;
						}
						if ( tdepth1<RELATIVE_SMALL_CONSTANT ) {
							free_flag = false;
							break;
						}
						if ( (depth_tol+voxel_center_depth) > tdepth1 ) {
							free_flag = false;
							break;
						}
					}
					if ( free_flag==false ) {
						break;
					}
				}

				if ( free_flag ) {
					++ (vdata[iz*numel_xy+iy*vol_res[0]+ix].free_vote);
				}

				if ( (depth0>RELATIVE_SMALL_CONSTANT) ) {
					float tdepth_diff = depth0-voxel_center_depth;
					if ( MY_ABS(tdepth_diff) < depth_tol ) {
						++ (vdata[iz*numel_xy+iy*vol_res[0]+ix].occp_vote);
					}
				}
			}
		}
	}

	return 0;
}

int CVolumeOccupancyEngine::ComputeVolumeOccupancy( CVolumeDataFloat* volume_data, fileio::DepthfileSimple* depthdata, float* transform_matrix, float depth_scale ) {
  
	float* bbox = (float*)(volume_data->BBox());
	int* vol_res = (int*)(volume_data->VolRes());
	voxelf* vdata = volume_data->vdata;
	int dwidth = depthdata->dwidth;
	int dheight = depthdata->dheight;
	int dwidth_m1 = dwidth - 1;
	int dheight_m1 = dheight - 1;
  
	float xdis = bbox[3]-bbox[0];
	float ydis = bbox[4]-bbox[1];
	float zdis = bbox[5]-bbox[2];
  
	float xunitdis = xdis/vol_res[0];
	float yunitdis = ydis/vol_res[1];
	float zunitdis = zdis/vol_res[2];
  
	//my_printf( 2, "xyz_units [%f,%f,%f]\n", xunitdis, yunitdis, zunitdis );
  
	float xhalfunitdis = xunitdis/2;
	float yhalfunitdis = yunitdis/2;
	float zhalfunitdis = zunitdis/2;
  
	float voxel_center[3];
	float voxel_proj[3];
  
	long numel_depth = ((long)dwidth)*dheight;
	float max_depth = 0;
	for ( long i=0; i<numel_depth; ++i ) {
		max_depth = MY_MAX( max_depth, depthdata->depthdata[i] );
	}
	//float depth_tol = max_depth*0.0002f;
	//float depth_tol = zunitdis;
	//float depth_tol = sqrt(xunitdis*xunitdis+yunitdis*yunitdis+zunitdis*zunitdis)/2;
	
#ifdef USING_CAM_POINT_DISTANCE
	float cam_center[3];
	transform_matrix_to_camera_pos( transform_matrix, cam_center );
	float depth_tol = sqrt(xunitdis*xunitdis+yunitdis*yunitdis+zunitdis*zunitdis)/2;
#else
	float depth_tol = zunitdis/2;
#endif
  
	long numel_xy = ((long)vol_res[0])*vol_res[1];
	for ( int iz=0; iz<vol_res[2]; ++iz ) {
		//my_printf( 3, "iz: %d/%d\n", iz+1, vol_res[2] );
		voxel_center[2] = bbox[2]+iz*zunitdis+zhalfunitdis;
		for ( int iy=0; iy<vol_res[1]; ++iy ) {
			voxel_center[1] = bbox[1]+iy*yunitdis+yhalfunitdis;
			for ( int ix=0; ix<vol_res[0]; ++ix ) {
				voxel_center[0] = bbox[0]+ix*xunitdis+xhalfunitdis;
				
				transform_matrix_pos_product( transform_matrix, voxel_center, voxel_proj );
				if ( voxel_proj[2]<RELATIVE_SMALL_CONSTANT ) {
					continue;
				}
				float xproj = voxel_proj[0]/voxel_proj[2]*depth_scale;
				float yproj = voxel_proj[1]/voxel_proj[2]*depth_scale;
				if ( (xproj<0)||(xproj>=dwidth_m1)||(yproj<0)||(yproj>=dheight_m1) ) {		// out of image boundary, skip
					continue;
				}
#ifdef USING_CAM_POINT_DISTANCE
				float voxel_center_depth = p3d_dis( voxel_center, cam_center );
#else
				float voxel_center_depth = voxel_proj[2];
#endif
        
				// simple closest point approach
				bool free_flag = true;
        float free_val = 1;
				float depth0 = 0;
        // pick the worst case (most unfree) scenario, be conservative
				for ( int dy=0; dy<2; ++ dy ) {
					for ( int dx=0; dx<2; ++ dx ) {
						//float tdepth1 = (depthdata->depthdata)[ (int)(xproj)+dx + ((int)(yproj)+dy)*dwidth ];
						float tdepth1 = (depthdata->depthdata)[ ((int)(xproj))+dx + (dheight_m1-((int)(yproj))+dy)*dwidth ];
						if ( dy==0 && dx==0 ) {
							depth0 = tdepth1;
						}
						if ( tdepth1<RELATIVE_SMALL_CONSTANT ) {
							free_flag = false;
              free_val = 0;
							break;
						}
						if ( (depth_tol+voxel_center_depth) > tdepth1 ) {
							free_flag = false;
              free_val = 0;
							break;
						}
            
            float tfratio = (voxel_center_depth)/tdepth1;
            if (tfratio>0.995) {
              free_flag = false;
              free_val = 0;
              break;
            }
            if (tfratio>0.895) {
              float tfval = 1-(tfratio-0.895)*10;
              free_val = MY_MIN(free_val, tfval);
            }
					}
					if ( free_flag==false ) {
						break;
					}
				}
        
				if ( free_flag ) {
					(vdata[iz*numel_xy+iy*vol_res[0]+ix].free_vote) += free_val;
				}
        
        if ( (depth0>RELATIVE_SMALL_CONSTANT) ) {
					float tdepth_diff = depth0-voxel_center_depth;
					if ( MY_ABS(tdepth_diff) < depth_tol ) {
						(vdata[iz*numel_xy+iy*vol_res[0]+ix].occp_vote) += 1;
					}
				}
			}
		}
	}
  
	return 0;
}
  
} // namespace freespace
