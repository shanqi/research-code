// Author: Qi Shan <shanqi@cs.washington.edu>

#include <iostream>
#include <fstream>
#include <string>

#include "common/common.h"
#include "fileio/depthfile_simple.h"

#include "freespace/VolumeOccupancyManager.h"
#include "freespace/VFS_ConfigureFile.h"
#include "freespace/VolumeOccupancyEngine.h"
#include "freespace/VolumeData.h"

using namespace std;

namespace freespace {

CVolumeOccupancyManager::CVolumeOccupancyManager(void){
	this->Init();
}

CVolumeOccupancyManager::CVolumeOccupancyManager(char* cfgfilename) {
	this->Init();
	this->Init(cfgfilename);
}

CVolumeOccupancyManager::~CVolumeOccupancyManager(void){
	this->destroy();
}

int CVolumeOccupancyManager::Init() {
	ready_flag = 0;
	memset( transform_matrix, 0, sizeof(float)*12 );
	vol_cfgfile = NULL;
	volume_data = NULL;

	return 0;
}

void CVolumeOccupancyManager::destroy() {
	SAFE_DELETE( vol_cfgfile );
	SAFE_DELETE( volume_data );
	vol_cfgfile = NULL;
	volume_data = NULL;
}

int CVolumeOccupancyManager::Init(char* cfgfilename) {
	my_printf( 4, "Initializing VolumeOccupancyManager...\n" );

	this->destroy();
	this->Init();
	my_printf( 2, "Loading configuration file %s\n", cfgfilename );
	this->vol_cfgfile = new CVFS_ConfigureFile(cfgfilename);
	if ( this->vol_cfgfile->ReadyFlag() == 1 ) {
		my_printf( 4, "Allocating volume data...\n" );
		this->volume_data = new CVolumeData();
		this->volume_data->VolInit(	this->vol_cfgfile->VolRes(), this->vol_cfgfile->Plybbox() );
		if ( this->volume_data->ReadyFlag() == 0 ) {	// something's wrong
			my_printf( 1, "VolumeOccupancyManager: not enough memory\n" );
			this->destroy();
			exit(-1);
		}
	} else {
		this->destroy();
	}
	this->ready_flag = 1;

	return 0;
}

int CVolumeOccupancyManager::LoadCameraProjMatFile( char* filename, float* ret_mat ) {
	if ( (ret_mat==NULL)||(filename==NULL)||(strlen(filename)==0) ) {
		return -2;
	}
	my_printf( 4, "Loading camera...\n" );
	string line;
	ifstream camfile (filename,ios::in);

	if (camfile.is_open()) {
		getline (camfile,line);
		for ( int i=0; i<3; ++i ) {
			getline (camfile,line);
			char* p = strtok( (char *)(line.c_str()), " " );
			int j=4*i;
			this->transform_matrix[j] = (float)atof(p);
			p = strtok( NULL, " " );
			this->transform_matrix[j+1] = (float)atof(p);
			p = strtok( NULL, " " );
			this->transform_matrix[j+2] = (float)atof(p);
			p = strtok( NULL, " " );
			this->transform_matrix[j+3] = (float)atof(p);
		}
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	}
	my_printf( 4, "Loading camera done.\n" );
	return 0;

}

int CVolumeOccupancyManager::WriteVolume( char* cfgfilename ) {

	char vfilename[MAX_FILE_NAME_LENGTH];
	sprintf( vfilename, "%s.volume", cfgfilename );
	this->volume_data->WriteFile( vfilename );

	return 0;
}

int CVolumeOccupancyManager::WriteVolumeFull( char* cfgfilename ) {

	char vfilename[MAX_FILE_NAME_LENGTH];
	sprintf( vfilename, "%s.volume", cfgfilename );
	this->volume_data->WriteFileFull( vfilename );

	return 0;
}

CVolumeData* CVolumeOccupancyManager::LoadVolumeUChar( char* filename ) {

	CVolumeData* retvol = new CVolumeData();
	retvol->LoadFileUChar( filename );

	return retvol;
}

CVolumeData* CVolumeOccupancyManager::LoadVolume( char* filename ) {

	CVolumeData* retvol = new CVolumeData();
	retvol->LoadFile( filename );

	return retvol;
}

int CVolumeOccupancyManager::WriteVolumeUChar( CVolumeData* input_voldata, char* filename, int color_scale ) {

	int retval = input_voldata->WriteFileUChar( filename, color_scale );

	return retval;
}

int CVolumeOccupancyManager::ComputingPlyVolumeOccupancy( float* pdata, long pnum ) {

	if ( this->ready_flag == 0 ) {
		return -1;
	}

	CVolumeOccupancyEngine::ComputePlyVolumeOccupancy( this->volume_data, pdata, pnum );

	return 0;
}

int CVolumeOccupancyManager::ComputingVolumeOccupancy() {
	if ( this->ready_flag == 0 ) {
		return -1;
	}

	my_printf( 2, "depth_scale %f\n", this->vol_cfgfile->depth_map_scale );

	int num_views = this->vol_cfgfile->NumViewpoints();
	int* viewid_list = this->vol_cfgfile->ViewPoints();
	const char* depthfiledir = this->vol_cfgfile->DepthfileDir();
	const char* depthfileprefix = this->vol_cfgfile->DepthfilePrefix();
	const char* camfiledir = this->vol_cfgfile->CamfileDir();
	//int* vol_res = this->vol_cfgfile->VolRes();
	//float* bbox = this->vol_cfgfile->Plybbox();

	for ( int iview=0; iview<num_views; ++iview ) {
		my_printf( 4, "Viewpoint %d\n", iview );
		int tviewid = viewid_list[iview];
		char depthfilename[MAX_FILE_NAME_LENGTH];
		char camfilename[MAX_FILE_NAME_LENGTH];
		sprintf( depthfilename, "%s/%s%08d.depth", depthfiledir, depthfileprefix, tviewid );
		sprintf( camfilename, "%s/%08d.txt", camfiledir, tviewid );

		my_printf( 2, "%s; %s\n", depthfilename, camfilename );

		int ret_test = this->LoadCameraProjMatFile( camfilename, this->transform_matrix );
		//my_printf( 2, "%f %f %f %f\n", transform_matrix[0], transform_matrix[1], transform_matrix[2], transform_matrix[3] );
		//my_printf( 2, "%f %f %f %f\n", transform_matrix[4], transform_matrix[5], transform_matrix[6], transform_matrix[7] );
		//my_printf( 2, "%f %f %f %f\n", transform_matrix[8], transform_matrix[9], transform_matrix[10], transform_matrix[11] );
		if ( ret_test<0 ) {
			my_printf( 1, "Cannot open file %s\n", camfilename );
			exit(-1);
		}

		fileio::DepthfileSimple depthdata( depthfilename );
		if ( depthdata.ready_flag != 1 ) {
			my_printf( 1, "Cannot open file %s\n", depthfilename );
			exit(-1);
		}
		double test_nonzeroratio = depthdata.NonZeroRatio();

		if ( test_nonzeroratio < 0.01 ) {
			continue;
		}
		CVolumeOccupancyEngine::ComputeVolumeOccupancy( this->volume_data, &depthdata, this->transform_matrix, this->vol_cfgfile->depth_map_scale );
	}

	return 0;
}
  
} // namespace freespace

