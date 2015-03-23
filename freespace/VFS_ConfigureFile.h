// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef FREESPACE_VFS_CONFIGURATION_FILE_H_
#define FREESPACE_VFS_CONFIGURATION_FILE_H_

#include "common/common.h"

namespace freespace {

class CVFS_ConfigureFile
{
public:
	CVFS_ConfigureFile(void);
	virtual ~CVFS_ConfigureFile(void);
	CVFS_ConfigureFile(char* filename);

private:
	void init();
	void destroy();

protected:
	int ready_flag;
	float plybbox[6];
	string depthfile_dir;
	string camfile_dir;
	string depthfile_prefix;
	int vol_res[3];
	int num_viewpoints;
	int* viewpoints;

public:
	float depth_map_scale;

public:
	float* Plybbox();
	const char* DepthfileDir();
	const char* CamfileDir();
	const char* DepthfilePrefix();
	int* VolRes();
	int NumViewpoints();
	int* ViewPoints();
	int ReadyFlag();

public:
	int LoadFile( char* filename );

public:
	static int MergePlybbox( float* bbox_merge, float* bbox_sub );
	
};
  
} // namespace freespace

#endif  // FREESPACE_VFS_CONFIGURATION_FILE_H_


