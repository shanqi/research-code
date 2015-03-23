// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef MICBOT_FILEIO_DEPTHFILE_SIMPLE_H_
#define MICBOT_FILEIO_DEPTHFILE_SIMPLE_H_

#include "common/common.h"

namespace fileio {

class DepthfileSimple {
  
public:
	DepthfileSimple(void);
	virtual ~DepthfileSimple(void);
	DepthfileSimple(char* filename);

private:
	void init();
	void destroy();

public:
	int ready_flag;
	int dwidth;
	int dheight;
	float* depthdata;

public:
	int LoadDepthFile( char* filename );
	int WriteDepthFile( char* filename );
	double NonZeroRatio();

};
  
}

#endif  // MICBOT_FILEIO_DEPTHFILE_SIMPLE_H_



