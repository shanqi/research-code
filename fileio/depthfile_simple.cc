// Author: Qi Shan <shanqi@cs.washington.edu>

#include "common/common.h"
#include "fileio/depthfile_simple.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

namespace fileio {

DepthfileSimple::DepthfileSimple(void){
	this->init();
}

DepthfileSimple::DepthfileSimple(char* filename){
	this->init();
	this->LoadDepthFile( filename );
}

DepthfileSimple::~DepthfileSimple(void){
	this->destroy();
}

void DepthfileSimple::init() {
	this->ready_flag = 0;
	this->dwidth = -1;
	this->dheight = -1;
	this->depthdata = NULL;
}

void DepthfileSimple::destroy() {
	SAFE_FREE( this->depthdata );
	this->depthdata = NULL;
	this->ready_flag = 0;
}

int DepthfileSimple::LoadDepthFile(char* filename) {
	
	this->destroy();
	this->init();

	string line;
	ifstream depthfile (filename,ios::in|ios::binary);
	if (depthfile.is_open()) {
		getline (depthfile,line);
		char* p = strtok( (char *)(line.c_str()), " " );
		this->dwidth = atoi(p);
		p = strtok( NULL, " " );
		this->dheight = atoi(p);
		int depth_size = dwidth*dheight;
		this->depthdata = (float*)malloc( sizeof(float)*depth_size );
		if ( this->depthdata == NULL ) {
			my_printf( 1, "DepthMap: not enough memory.\n" );
			depthfile.close();
			return -1;
		}
		memset( this->depthdata, 0, sizeof(float)*depth_size );
		depthfile.read( (char *)(this->depthdata), depth_size*sizeof(float));
		//if ( depthfile.fail() ) {
		//	my_printf( 1, "Error while reading file %s.\n", filename );
		//}
		ready_flag = 1;
		depthfile.close();
	} else {
		my_printf( 1, "Cannot open file %s .\n", filename );
		return -1;
	}

	return 0;
}

int DepthfileSimple::WriteDepthFile(char* filename) {

	if ( ready_flag==0 ) {
		return -2;
	}

	ofstream depthfile (filename,ios::out|ios::binary);
	if (depthfile.is_open()) {
		depthfile << this->dwidth << " " << this->dheight << "\n";
		depthfile.write( (char*)(this->depthdata), dwidth*dheight*sizeof(float) );
		depthfile.close();
	} else {
		my_printf( 1, "Cannot open file %s.\n", filename );
		return -1;
	} 

	return 0;
}

double DepthfileSimple::NonZeroRatio() {

	if ( ready_flag==0 ) {
		return -2;
	}

	long num_non_zeros = 0;
	long numel_xy = this->dheight*this->dwidth;
	for ( long i=0; i<numel_xy; ++i ) {
		float tval = this->depthdata[i];
		if ( MY_ABS( tval ) > RELATIVE_SMALL_CONSTANT ) {
			++ num_non_zeros;
		}
	}
	double ret_val = ((double)num_non_zeros)/numel_xy;
	return ret_val;
}
  
}   // namespace fileio
