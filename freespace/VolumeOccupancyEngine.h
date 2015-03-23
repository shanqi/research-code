// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef FREESPACE_VOLUME_OCCUPANCY_ENGINE_H_
#define FREESPACE_VOLUME_OCCUPANCY_ENGINE_H_

#include "common/common.h"
#include "fileio/depthfile_simple.h"

namespace freespace {

class CVolumeData;
class CVolumeDataFloat;

class CVolumeOccupancyEngine
{
public:
	CVolumeOccupancyEngine(void);
	virtual ~CVolumeOccupancyEngine(void);

public:
	static int ComputeVolumeOccupancy( CVolumeData* volume_data, fileio::DepthfileSimple* depthdata, float* transform_matrix, float depth_scale );
  static int ComputeVolumeOccupancy( CVolumeDataFloat* volume_data, fileio::DepthfileSimple* depthdata, float* transform_matrix, float depth_scale );
	static int ComputePlyVolumeOccupancy( CVolumeData* volume_data, float* pdata, long pnum );
  static int ComputePlyVolumeOccupancy( CVolumeDataFloat* volume_data, float* pdata, long pnum );

};
  
} // namespace freespace

#endif  // FREESPACE_VOLUME_OCCUPANCY_ENGINE_H_


