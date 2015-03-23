// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef MICBOT_FILEIO_FILEIO_SIMPLE_H_
#define MICBOT_FILEIO_FILEIO_SIMPLE_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <dirent.h>

#include "common/common.h"

namespace fileio {

void LoadFile(const char* filename, std::vector<std::string>* lines);
  
inline bool FileExists (const std::string& name) {
  if (FILE *file = fopen(name.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

template <class T>
int LoadDataFile1d(const char* filename, int* length, T** ret_data) {
  
  std::ifstream datafile(filename,std::ios::in|std::ios::binary);
  std::string line;
  if (datafile.is_open()) {
    datafile.seekg(0, datafile.end);
    mylong data_size = datafile.tellg();
    datafile.seekg (0, datafile.beg);
    
    *length = data_size/sizeof(T);
    *ret_data = new T[*length];
    if ( *ret_data == NULL ) {
      my_printf( 1, "LoadDataFile2d: not enough memory.\n" );
      datafile.close();
      return -1;
    }
    memset( *ret_data, 0, data_size );
    datafile.read( (char *)(*ret_data), data_size);
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to read: %s .\n", filename );
    return -1;
  }
  
  return 0;
}

template <class T>
int SaveDataFile1d(const char* filename, int length, T* data) {
  std::ofstream datafile(filename,std::ios::out|std::ios::binary);
  if (datafile.is_open()) {
    mylong data_size = length;
    data_size *= sizeof(T);
    datafile.write( (char*)(data), data_size );
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to write: %s.\n", filename );
    return -1;
  }
  return 0;
}

template <class T>
int LoadDataFile1dv(const char* filename, int* length, vector<T>& ret_data) {
  std::ifstream datafile(filename,std::ios::in|std::ios::binary);

  if (datafile.is_open()) {
    datafile.seekg(0, datafile.end);
    mylong data_size = datafile.tellg();
    datafile.seekg (0, datafile.beg);
    
    *length = data_size/sizeof(T);
    
    T buf = 0;
    for ( int j=0; j<(*length); ++j ) {
      datafile.read( (char *)(&buf), sizeof(T));
      ret_data.push_back( buf );
    }
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to read: %s .\n", filename );
    return -1;
  }
  return 0;
}

template <class T>
int SaveDataFile1dv(const char* filename, int length, vector<T>& data) {
  std::ofstream datafile(filename,std::ios::out|std::ios::binary);
  if (datafile.is_open()) {
    for (int j=0; j<length; ++j) {
      datafile.write( (char*)(&(data[j])), sizeof(T) );
    }
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to write: %s.\n", filename );
    return -1;
  }
  return 0;
}
  
int LoadDataFile1dvIntTxT(const char* filename, int* length, vector<int>& ret_data);
int SaveDataFile1dvIntTxt(const char* filename, int length, vector<int>& in_data);

template <class T>
int LoadDataFile2d(const char* filename, int* width, int* height,T** ret_data, bool silence=false) {
  
  std::ifstream datafile(filename,std::ios::in|std::ios::binary);
  std::string line;
  if (datafile.is_open()) {
    getline (datafile,line);
    char* p = strtok( (char *)(line.c_str()), " " );
    *width = atoi(p);
    p = strtok( NULL, " " );
    *height = atoi(p);
    int data_size = (*width)*(*height);
    *ret_data = new T[data_size];
    if ( *ret_data == NULL ) {
      if (!silence) {
        my_printf( 1, "LoadDataFile2d: not enough memory.\n" );
      }
      datafile.close();
      return -1;
    }
    memset( *ret_data, 0, sizeof(T)*data_size );
    datafile.read( (char *)(*ret_data), data_size*sizeof(T));
    datafile.close();
  } else {
    if (!silence) {
      my_printf( 1, "Cannot open file to read: %s .\n", filename );
    }
    return -1;
  }
  
  return 0;
}
  
template <class T>
int SaveDataFile2d(const char* filename, int width, int height, T* data) {
  std::ofstream datafile(filename,std::ios::out|std::ios::binary);
  if (datafile.is_open()) {
    datafile << width << " " << height << "\n";
    datafile.write( (char*)(data), width*height*sizeof(T) );
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to write: %s.\n", filename );
    return -1;
  }
  return 0;
}
  
template <class T>
int SaveDataFile3d(const char* filename, mylong width, mylong height, mylong size_d3, T* data) {
  std::ofstream datafile(filename,std::ios::out|std::ios::binary);
  if (datafile.is_open()) {
    datafile << width << " " << height << " " << size_d3 <<"\n";
    datafile.write( (char*)(data), width*height*size_d3*sizeof(T) );
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to write: %s.\n", filename );
    return -1;
  }
  return 0;
}

    
#define LoadDataFile2dc LoadDataFile2c<double>
#define LoadDataFile2df LoadDataFile2d<float>
#define LoadDataFile2dd LoadDataFile2d<double>
    
int ReadDir(string dirname, vector<string>& file_list);

inline bool str_ends_with(const char* str, const char* suffix) {
  if (str == NULL || suffix == NULL) {
    return 0;
  }
  
  size_t str_len = strlen(str);
  size_t suffix_len = strlen(suffix);
  
  if (suffix_len > str_len) return 0;
  
  return (0 == strncmp(str + str_len - suffix_len, suffix, suffix_len));
}
  
inline bool string_ends_with(string str, string suffix) {
  return str_ends_with(str.c_str(), suffix.c_str());
}
  
inline int number_of_files(string filename_exp) {
  //an interesting way of getting the number of files in a folder
  int num_depth_files = 0;
  FILE* num_images_in_popen;
  char command_line[1024];
  sprintf(command_line, "ls -l %s/*.depth|wc -l\n", filename_exp.c_str() );
  if ((num_images_in_popen = popen(command_line, "r"))) {
    char buff[512];
    if (fgets(buff, sizeof(buff), num_images_in_popen) != NULL ) {
      num_depth_files = atoi(buff);
    }
  }
  return num_depth_files;
}
  
inline bool LoadModelCenter(string filename, double* model_center) {
  
  vector<string> lines;
  fileio::LoadFile(filename.c_str(), &lines);
  vector<string> nums = StringSplit(lines[0], " ");
  model_center[0] = atof( nums[0].c_str() );
  model_center[1] = atof( nums[1].c_str() );
  model_center[2] = atof( nums[2].c_str() );
  
  return true;
}
  
inline bool is_number(const std::string& s) {
  std::string::const_iterator it = s.begin();
  while (it != s.end() && (std::isdigit(*it)||(*it=='-')||(*it=='+')||(*it=='.')||(*it=='\n'))) {
    ++ it;
  }
  bool ret = (!s.empty()) && (it==s.end());
  cout << "is_number " << s << " " << ret << endl;
  return ret;
}
  
inline bool LoadSimTransParameters(string filename, double* sim_parameters) {
  
  vector<string> lines;
  fileio::LoadFile(filename.c_str(), &lines);
  int num_lines = lines.size();
  int ncounter = 0;
  for (int j=0; j<num_lines; ++j) {
    vector<string> nums = StringSplit(lines[j], " ");
    int num_nums = nums.size();
    for (int k=0; k<num_nums; ++k) {
      if (is_number(nums[k])) {
        sim_parameters[ncounter] = atof( nums[k].c_str() );
        ++ ncounter;
      }
    }
  }
  
  return true;
}

}

#endif  // MICBOT_FILEIO_FILEIO_SIMPLE_H_
