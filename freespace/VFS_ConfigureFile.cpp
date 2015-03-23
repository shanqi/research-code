// Author: Qi Shan <shanqi@cs.washington.edu>

#include "freespace/VFS_ConfigureFile.h"
#include "common/common.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

namespace freespace {

CVFS_ConfigureFile::CVFS_ConfigureFile(void){
	init();
}

CVFS_ConfigureFile::CVFS_ConfigureFile(char* filename){
	init();
	this->LoadFile(filename);
}

CVFS_ConfigureFile::~CVFS_ConfigureFile(void){
	this->destroy();
}

void CVFS_ConfigureFile::init() {
	this->ready_flag = 0;
	memset(this->plybbox, 0, sizeof(float)*6);
	memset(this->vol_res, 0, sizeof(int)*3);

	this->camfile_dir.clear();
	this->depthfile_dir.clear();
	this->depthfile_prefix.clear();

	this->num_viewpoints = 0;
	this->viewpoints = NULL;

	this->depth_map_scale = 1.0f;

}

void CVFS_ConfigureFile::destroy() {
	SAFE_FREE( this->viewpoints );
	this->viewpoints = NULL;
}

//#define DEBUG_CVFS_ConfigureFile_LOADFILE

int CVFS_ConfigureFile::LoadFile(char* filename) {
	my_printf(4,"Loading Configuration file %s...\n", filename);
	this->destroy();
	this->init();

	string line;
	ifstream configfile (filename,ios::in|ios::binary);
	if (configfile.is_open()) {
		while (1) {
			getline (configfile,line);
			if ( configfile.eof() || configfile.fail() ) {
				break;
			}
			if ( line.length() == 0 ) {
				continue;
			}
			char* p = strtok( (char *)(line.c_str()), "=" );
#ifdef DEBUG_CVFS_ConfigureFile_LOADFILE
			my_printf(2,"%s\n", p);
#endif
			if ( strcmp( p, "plybbox" ) == 0 ) {
				p = strtok( NULL, "[" );
				p = strtok( p, "," );
				this->plybbox[0] = (float)(atof(p));
				p = strtok( NULL, "," );
				this->plybbox[1] = (float)(atof(p));
				p = strtok( NULL, ";" );
				this->plybbox[2] = (float)(atof(p));
				p = strtok( NULL, "," );
				this->plybbox[3] = (float)(atof(p));
				p = strtok( NULL, "," );
				this->plybbox[4] = (float)(atof(p));
				p = strtok( NULL, "]" );
				this->plybbox[5] = (float)(atof(p));
			} else if ( strcmp( p, "depthfile_dir" ) == 0 ) {
				p = strtok( NULL, "=" );
				this->depthfile_dir.clear();
				this->depthfile_dir.append( p );
			} else if ( strcmp( p, "depthfile_prefix" ) == 0 ) {
				p = strtok( NULL, "=" );
#ifdef DEBUG_CVFS_ConfigureFile_LOADFILE
				if ( p==NULL ) {
					my_printf(2,"p==NULL\n");
				}
#endif
				if ( p!=NULL && strlen(p)>1 ) {
					this->depthfile_prefix.clear();
					this->depthfile_prefix.append( p );
				} else {
					this->depthfile_prefix.clear();
					this->depthfile_prefix.append( "" );
				}
			} else if ( strcmp(p, "depth_map_scale") == 0 ) {
				p = strtok( NULL, "=" );
				if ( p!=NULL && strlen(p)>1 ) {
					this->depth_map_scale = atof(p);
				}
			} else if ( strcmp( p, "camfile_dir" ) == 0 ) {
				p = strtok( NULL, "=" );
				this->camfile_dir.clear();
				this->camfile_dir.append(p);
			} else if ( strcmp( p, "volume_res" ) == 0 ) {
				p = strtok( NULL, "[" );
				p = strtok( p, "," );
				this->vol_res[0] = atoi(p);
				p = strtok( NULL, "," );
				this->vol_res[1] = atoi(p);
				p = strtok( NULL, "]" );
				this->vol_res[2] = atoi(p);
			} else if ( strcmp( p, "viewpoints" ) == 0 ) {
				p = strtok( NULL, "=" );
				p = strtok( p, " " );				
				this->num_viewpoints = atoi(p);
				this->viewpoints = (int*)malloc( sizeof(int)*(this->num_viewpoints) );
				for ( int i=0; i<this->num_viewpoints; ++i ) {
					p = strtok( NULL, " " );
					this->viewpoints[i] = atoi(p);
				}
			} else {
				my_printf( 1, "Unknown property: %s.\n", p );
			}
		}
		ready_flag = 1;
		configfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	}

	return 0;
}

int CVFS_ConfigureFile::MergePlybbox( float* bbox_merge, float* bbox_sub ) {

	for ( int i=0; i<3; ++i ) {
		bbox_merge[i] = MY_MIN( (bbox_merge[i]), (bbox_sub[i]) );
	}
	for ( int i=3; i<6; ++i ) {
		bbox_merge[i] = MY_MAX( (bbox_merge[i]), (bbox_sub[i]) );
	}

	return 0;
}

float* CVFS_ConfigureFile::Plybbox() {
	return this->plybbox;
}

const char* CVFS_ConfigureFile::DepthfileDir() {
	return this->depthfile_dir.c_str();
}

const char* CVFS_ConfigureFile::CamfileDir() {
	return this->camfile_dir.c_str();
}

const char* CVFS_ConfigureFile::DepthfilePrefix() {
	return this->depthfile_prefix.c_str();
}

int* CVFS_ConfigureFile::VolRes() {
	return this->vol_res;
}

int CVFS_ConfigureFile::NumViewpoints() {
	return this->num_viewpoints;
}

int* CVFS_ConfigureFile::ViewPoints() {
	return this->viewpoints;
}

int CVFS_ConfigureFile::ReadyFlag() {
	return this->ready_flag;
}
  
} // namespace freespace

