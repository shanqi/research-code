// Author: Qi Shan <shanqi@cs.washington.edu>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <dirent.h>

#include "common/common.h"
#include "fileio/fileio_simple.h"

using namespace std;

namespace fileio {
  
void LoadFile(const char* filename, std::vector<std::string>* lines){
	std::string line;
	ifstream myfile (filename);
	if (myfile.is_open()) {
		while ( getline (myfile,line) ) {
			lines->push_back( line );
		}
		myfile.close();
	}
}
    
int ReadDir(string dirname, vector<string>& file_list) {
    
    DIR *dirp;
    dirp = opendir(dirname.c_str());
    struct dirent* dp;
    while ((dp = readdir(dirp)) != NULL) {
        file_list.push_back(string(dp->d_name));
    }
    (void)closedir(dirp);

    return file_list.size();
}
  
int LoadDataFile1dvIntTxT(const char* filename, int* length, vector<int>& ret_data) {
  std::ifstream datafile(filename,std::ios::in);
  std::string line;
  if (datafile.is_open()) {
    getline (datafile,line);
    //mylog(2, line.c_str());
    char* p = strtok( (char *)(line.c_str()), " " );
    while (p != NULL) {
      if (strlen(p)>0) {
        int tnum = atoi(p);
        ret_data.push_back( tnum );
        //mylog(2, "%d ", tnum);
      }
      p = strtok( NULL, " " );
    }
    //mylog(2, "\n");
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to read: %s .\n", filename );
    return -1;
  }
  return 0;
}

int SaveDataFile1dvIntTxt(const char* filename, int length, vector<int>& in_data) {
  std::ofstream datafile(filename,std::ios::out);
  if (datafile.is_open()) {
    for (int j=0; j<length; ++j) {
      datafile << in_data[j];
      if (j!=length-1) {
        datafile << " ";
      } else {
        datafile << "\n";
      }
    }
    datafile.close();
  } else {
    my_printf( 1, "Cannot open file to write: %s.\n", filename );
    return -1;
  }
  return 0;
}

}