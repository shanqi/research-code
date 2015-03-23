// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef MICBOT_FILEIO_RUN_LENGTH_ENCODING_H_
#define MICBOT_FILEIO_RUN_LENGTH_ENCODING_H_

#include "common/common.h"

namespace fileio {
  namespace rle {

typedef struct stream_t stream_t, *stream;
struct stream_t {
	int (*get)(stream);
	int (*put)(stream, int);
};
 
typedef struct {
	int (*get)(stream);
	int (*put)(stream, int);
	char *string;
	int pos;
} string_stream;

typedef struct {
	int (*get)(stream);
	int (*put)(stream, int);
	unsigned char *string;
	int capacity;
	int pos;
	int string_len;
} byte_stream;
 
typedef struct {
	int (*get)(stream);
	int (*put)(stream, int);
	FILE *fp;
} file_stream;

int sget(stream in);
int sput(stream out, int c);
int bget(stream in);
int bput(stream out, int c);
int file_put(stream out, int c);
void output(stream out, unsigned char* buf, int len);
void encode(stream in, stream out);
void decode(stream in, stream out);
  
  }	// namespace rle
}	// namespace fileio

#endif  // MICBOT_FILEIO_RUN_LENGTH_ENCODING_H_

