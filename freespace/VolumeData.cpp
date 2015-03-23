// Author: Qi Shan <shanqi@cs.washington.edu>

#include "freespace/VolumeData.h"
#include "common/common.h"
#include "fileio/rle.h"

#include <iostream>
#include <fstream>
#include <string>
#include <climits>

using namespace std;
using namespace fileio::rle;

namespace freespace {

CVolumeData::CVolumeData(void) {
	init();
}

CVolumeData::~CVolumeData(void) {
	destroy();
}

void CVolumeData::init() {
	this->vdata = NULL;
	this->ready_flag = 0;
}

void CVolumeData::destroy() {
	SAFE_FREE( this->vdata );
	this->vdata = NULL;
}

int CVolumeData::VolInit( int* in_vol_res, float* in_bbox ) {

	this->destroy();
	this->init();

	long numel_vol = ((long)in_vol_res[0]) * in_vol_res[1] * in_vol_res[2];
	this->vdata = (voxel*)malloc( sizeof(voxel)*numel_vol );
	if ( this->vdata==NULL ) {
		my_printf( 1, "Not enough memory [%ld]\n", numel_vol );
		return -1;
	}
	my_printf( 3, "Volres init: [%d,%d,%d]. ", in_vol_res[0], in_vol_res[1], in_vol_res[2] );
	my_printf( 3, "Successfully allocated %f G space.\n", (sizeof(voxel))*numel_vol/((float)1e9) );
	memset( this->vdata, 0, sizeof(voxel)*numel_vol );

	memcpy( this->vol_res, in_vol_res, sizeof(int)*3 );
	memcpy( this->bbox, in_bbox, sizeof(float)*6 );

	this->ready_flag = 1;

	return 0;

}

int CVolumeData::WriteFileUChar( char* filename, int color_scale ) {

	ofstream volfile (filename,ios::out|ios::binary);
	if (volfile.is_open()) {
		volfile.write( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		unsigned char* buf = (unsigned char*)malloc( sizeof(unsigned char)*numel_xy );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {
			memset( buf, 0, sizeof(unsigned char)*numel_xy );
			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				if ( pvdata[k].free_vote > 0 ) {
					int tval = color_scale*pvdata[k].free_vote;
					buf[k] = MY_TRUNCATE( tval, 0, UCHAR_MAX );
				}
				//if ( pvdata[k].free_vote >= FREE_VOTE_THRESHOLD ) {
				//	buf[k] = 0;
				//} else {
				//	buf[k] = UCHAR_MAX;
				//}
			}
			volfile.write( (char*)buf, numel_xy );
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::WriteFileUShort( char* filename ) {
	return 0;
}

int CVolumeData::MergePlyVolFile( char* filename ) {

	ifstream volfile (filename,ios::in|ios::binary);
	if (volfile.is_open()) {
		int tvol_res[3];
		volfile.read( (char*)(tvol_res), 3*sizeof(int)/sizeof(char) );
		
		mylong numel_xy = ((mylong)vol_res[0] )* vol_res[1];
		//mylong numel_xyz = numel_xy*vol_res[2];
		
		//memset( this->vdata, 0, (sizeof(voxel))*numel_xyz );
		unsigned short* buf = (unsigned short*)malloc( sizeof(unsigned short)*numel_xy );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {
			volfile.read( (char*)buf, numel_xy*sizeof(unsigned short) );
			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				pvdata[k].occp_vote = buf[k];
			}			
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::LoadFile( char* filename ) {

	this->destroy();

	ifstream volfile (filename,ios::in|ios::binary);
	if (volfile.is_open()) {
		volfile.read( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		long numel_xyz = numel_xy*vol_res[2];
		this->vdata = (voxel*)malloc( (sizeof(voxel))*numel_xyz );
		if ( this->vdata == NULL ) {
			my_printf( 1, "Not enough memory. (%f G)\n", (sizeof(voxel))*numel_xyz/((float)1e9) );
			return -2;
		}
		my_printf( 3, "Volres: [%d,%d,%d]. ", this->vol_res[0], this->vol_res[1], this->vol_res[2] );
		my_printf( 3, "Successfully allocated %f G space.\n", (sizeof(voxel))*numel_xyz/((float)1e9) );
		memset( this->vdata, 0, (sizeof(voxel))*numel_xyz );
		unsigned short* buf = (unsigned short*)malloc( sizeof(unsigned short)*numel_xy );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {
			volfile.read( (char*)buf, numel_xy*sizeof(unsigned short) );

			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				pvdata[k].free_vote = buf[k];
			}
			
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::LoadFileFull( char* filename ) {

	this->destroy();

	ifstream volfile (filename,ios::in|ios::binary);
	if (volfile.is_open()) {
		volfile.read( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		long numel_xyz = numel_xy*vol_res[2];
		this->vdata = (voxel*)malloc( (sizeof(voxel))*numel_xyz );
		if ( this->vdata == NULL ) {
			my_printf( 1, "Not enough memory. (%f G)\n", (sizeof(voxel))*numel_xyz/((float)1e9) );
			return -2;
		}
		my_printf( 3, "Volres: [%d,%d,%d]. ", this->vol_res[0], this->vol_res[1], this->vol_res[2] );
		my_printf( 3, "Successfully allocated %f G space.\n", (sizeof(voxel))*numel_xyz/((float)1e9) );
		memset( this->vdata, 0, (sizeof(voxel))*numel_xyz );

		volfile.read( (char*)(this->vdata), numel_xyz*(sizeof(voxel)) );
		
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::LoadFileUChar( char* filename ) {

	this->destroy();

	ifstream volfile (filename,ios::in|ios::binary);
	if (volfile.is_open()) {
		volfile.read( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		long numel_xyz = numel_xy*vol_res[2];
		this->vdata = (voxel*)malloc( (sizeof(voxel))*numel_xyz );
		memset( this->vdata, 0, (sizeof(voxel))*numel_xyz );
		unsigned char* buf = (unsigned char*)malloc( sizeof(unsigned char)*numel_xy );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {
			volfile.read( (char*)buf, numel_xy*sizeof(unsigned char) );

			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				pvdata[k].free_vote = buf[k];
			}
			
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::WriteFileFull( char* filename ) {

	ofstream volfile (filename,ios::out|ios::binary);
	if (volfile.is_open()) {
		volfile.write( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {			
			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			volfile.write( (char*)pvdata, numel_xy*sizeof(voxel) );
		}
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::WriteFile( char* filename ) {

	ofstream volfile (filename,ios::out|ios::binary);
	if (volfile.is_open()) {
		volfile.write( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		unsigned short* buf = (unsigned short*)malloc( sizeof(unsigned short)*numel_xy );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {			
			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				buf[k] = pvdata[k].free_vote;
			}
			volfile.write( (char*)buf, numel_xy*sizeof(unsigned short) );
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::WriteFileRLE( char* filename, long& total_length ) {

	total_length = 0;
	ofstream volfile (filename,ios::out|ios::binary);
	if (volfile.is_open()) {
		volfile.write( (char*)(this->vol_res), 3*sizeof(int)/sizeof(char) );
		long numel_xy = ((long)vol_res[0] )* vol_res[1];
		unsigned short* buf = (unsigned short*)malloc( sizeof(unsigned short)*numel_xy );
		unsigned char* compbuf = (unsigned char*)malloc( sizeof(unsigned short)*numel_xy*2 );
		for ( int iz=0; iz < vol_res[2]; ++ iz ) {
			voxel* pvdata = &(this->vdata[iz*numel_xy]);
			for ( long k=0; k<numel_xy; ++k ) {
				buf[k] = pvdata[k].free_vote;
			}
			
			byte_stream str_in = { bget, 0, (unsigned char*)buf, (int)(numel_xy*sizeof(unsigned short)), 0, (int)(numel_xy*sizeof(unsigned short))};
			byte_stream str_out = { bget, bput, compbuf, (int)(numel_xy*sizeof(unsigned short)*2), 0, (int)(numel_xy*sizeof(unsigned short)*2)};
			/* encode from str_in to str_out */
			encode((stream)&str_in, (stream)&str_out);

			int compr_len = str_out.string_len;
			total_length += compr_len;
			volfile.write( (const char*)compbuf, compr_len );
			//volfile.write( (char*)buf, numel_xy*sizeof(unsigned short) );
		}
		SAFE_FREE( buf );
		volfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

int CVolumeData::ReadyFlag() {
	return this->ready_flag;
}

const float* CVolumeData::BBox() {
	return this->bbox;
}

const int* CVolumeData::VolRes() {
	return this->vol_res;
}

long CVolumeData::VolumeFreeVoteSum() {
	long retval = 0;

	long numel_xyz = ((long)vol_res[0] )* vol_res[1]*vol_res[2];
	for ( long i=0; i<numel_xyz; ++i ) {
		retval += this->vdata[i].free_vote;
	}

	return retval;
}

int CVolumeData::MergePlyVol( CVolumeData* plyvol ) {

	long xdim = vol_res[0];
	long ydim = vol_res[1];
	long zdim = vol_res[2];
	long numel_xy = xdim*ydim;

	long hwin = 1;

	for ( long iz=0; iz<zdim; ++iz ) {
		long zstart = iz-hwin;		
		long zend = iz+hwin;
		zstart = (zstart < 0) ? 0 : zstart;
		zend = (zend >= zdim) ? zdim-1 : zend;

		for ( long iy=0; iy<ydim; ++iy ) {
			long ystart = iy-hwin;			
			long yend = iy+hwin;
			ystart = ystart < 0 ? 0 : ystart;
			yend = yend >= ydim ? ydim-1 : yend;
			for ( long ix=0; ix<xdim; ++ ix ) {
				unsigned short tplyvote = plyvol->vdata[ iz*numel_xy + iy*xdim + ix ].free_vote;
				if ( tplyvote > 0 ) {
					long xstart = ix-hwin;
					xstart = xstart < 0 ? 0 : xstart;
					long xend = ix+hwin;
					xend = xend >= xdim ? xdim-1 : xend;

					for ( long tz=zstart; tz<=zend; ++tz ) {
						for ( long ty=ystart; ty<=yend; ++ty ) {
							for ( long tx=xstart; tx<=xend; ++ tx ) {
								this->vdata[ tz*numel_xy + ty*xdim + tx ].occp_vote += tplyvote;
							}
						}
					}
				}
			}
		}
	}

	return 0;
}

int CVolumeData::MergePlyVolSimple( CVolumeData* plyvol ) {

	long numel_xyz = ((long)(this->vol_res[0]))*vol_res[1]*vol_res[2];

	voxel* pfreevox = this->vdata;
	voxel* pplyvox = plyvol->vdata;

	for ( long i=0; i<numel_xyz; ++i ) {
		pfreevox->occp_vote += pplyvox->free_vote;
		++ pfreevox;
		++ pplyvox;
	}

	return 0;
}

int CVolumeData::MergeSubVolume( CVolumeData* subvol, int ux, int uy, int uz, int* subvolres ) {

	long xskip = ux*subvolres[0];
	long yskip = uy*subvolres[1];
	long zskip = uz*subvolres[2];	

	for ( int iz=0; iz<subvolres[2]; ++ iz ) {
		long dz = iz+zskip;
		long izss = ((long)iz)*subvolres[1]*subvolres[0];
		long dzvv = ((long)dz)*(this->vol_res[1])*(this->vol_res[0]);

		for ( int iy=0; iy<subvolres[1]; ++iy ) {
			long dy = iy+yskip;
			voxel* pvsub = &(subvol->vdata[izss+((long)iy)*subvolres[0]]);
			voxel* pvmerge = &(this->vdata[dzvv+((long)dy)*(this->vol_res[0])+xskip]);

			for ( int ix=0; ix<subvolres[0]; ++ix ) {
				pvmerge[ix].free_vote += pvsub[ix].free_vote;
				pvmerge[ix].occp_vote += pvsub[ix].occp_vote;
			}
		}
	}

	return 0;
}
  
} // namespace freespace


