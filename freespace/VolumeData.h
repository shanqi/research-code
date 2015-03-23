// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef FREESPACE_VOLUME_DATA_H_
#define FREESPACE_VOLUME_DATA_H_

#define FREE_VOTE_THRESHOLD 3

namespace freespace {

typedef struct tag_VOXEL {
	unsigned short free_vote;
	unsigned short occp_vote;
} voxel;

class CVolumeData
{
public:
	CVolumeData(void);
	virtual ~CVolumeData(void);

public:
	int VolInit( int* in_vol_res, float* in_bbox );
	int LoadFile( char* filename );
	int LoadFileUChar( char* filename );
	int LoadFileFull( char* filename );

	int WriteFileUChar( char* filename, int color_scale );	
	int WriteFileUShort( char* filename );
	int WriteFile( char* filename );
	int WriteFileFull( char* filename );
	int WriteFileRLE( char* filename, long& total_length );

	int ReadyFlag();
	const float* BBox();
	const int* VolRes();

private:
	void destroy();
	void init();

protected:
	int ready_flag;
	int vol_res[3];
	float bbox[6];

public:
	voxel* vdata;

public:
	long VolumeFreeVoteSum();

public:
	int MergeSubVolume( CVolumeData* subvol, int ux, int uy, int uz, int* subvolres );

public:
	int MergePlyVolFile( char* filename );
	int MergePlyVol( CVolumeData* plyvol );
	int MergePlyVolSimple( CVolumeData* plyvol );
};
  
} // namespace freespace

#endif  // FREESPACE_VOLUME_DATA_H_


