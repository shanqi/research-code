// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef FREESPACE_VOLUME_OCCUPANCY_MANAGER_H_
#define FREESPACE_VOLUME_OCCUPANCY_MANAGER_H_

namespace freespace {

class CVFS_ConfigureFile;
class CVolumeData;

class CVolumeOccupancyManager
{
public:
	CVolumeOccupancyManager(void);
	CVolumeOccupancyManager(char* cfgfilename);
	virtual ~CVolumeOccupancyManager(void);

private:
	int Init();
	void destroy();

protected:
	int Init( char* cfgfilename );

public:
	int ready_flag;
	float transform_matrix[12];
	CVFS_ConfigureFile* vol_cfgfile;
	CVolumeData* volume_data;

public:
	int LoadCameraProjMatFile( char* filename, float* ret_mat );

public:
	int ComputingVolumeOccupancy();
	int ComputingPlyVolumeOccupancy( float* pdata, long pnum );
	int WriteVolume( char* cfgfilename );
	int WriteVolumeFull( char* cfgfilename );
	CVolumeData* LoadVolume( char* cfgfilename );
	CVolumeData* LoadVolumeUChar( char* filename );

	int WriteVolumeUChar( CVolumeData* input_voldata, char* filename, int color_scale=255 );

};
  
} // namespace freespace

#endif  // MODEL3D_VOLUME_OCCUPANCY_MANAGER_H_


