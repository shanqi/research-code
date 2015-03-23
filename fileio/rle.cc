// Author: Qi Shan <shanqi@cs.washington.edu>

#include <stdio.h>
#include <stdlib.h>
#include "fileio/rle.h"

namespace fileio {
namespace rle {

int bget(stream in) {
	int c;
	byte_stream* s = (byte_stream*) in;
	if ( (s->pos>=s->capacity) || (s->pos>=s->string_len) ) {
		return -1;
	}
	c = (s->string[s->pos]);
	s->pos++;
	return c;
}
 
int bput(stream out, int c) {
	byte_stream* s = (byte_stream*) out;
	if ( s->pos >= s->capacity ) {
		return -1;
	}
	s->string[s->pos++] = (c == -1) ? '\0' : c;
	s->string_len = s->pos - 1;
	if (c == -1) {
		s->pos = 0;
	}

	return 0;
}

int sget(stream in) {
	int c;
	string_stream* s = (string_stream*) in;
	c = (unsigned char)(s->string[s->pos]);
	if (c == '\0') return -1;
	s->pos++;
	return c;
}
 
int sput(stream out, int c) {
	string_stream* s = (string_stream*) out;
	s->string[s->pos++] = (c == -1) ? '\0' : c;
	if (c == -1) s->pos = 0;
	return 0;
}
 
int file_put(stream out, int c) {
	file_stream *f = (file_stream*) out;
	return fputc(c, f->fp);
}
 
/* helper function */
void output(stream out, unsigned char* buf, int len) {
	int i;
	out->put(out, 128 + len);
	for (i = 0; i < len; i++) {
		out->put(out, buf[i]);
	}
}
 
void encode(stream in, stream out) {
	unsigned char buf[256];
	int len = 0, repeat = 0, end = 0, c;
	int (*get)(stream) = in->get;
	int (*put)(stream, int) = out->put;
 
	while (!end) {
		end = ((c = get(in)) == -1);
		if (!end) {
			buf[len++] = c;
			if (len <= 1) continue;
		}
 
		if (repeat) {
			if (buf[len - 1] != buf[len - 2])
				repeat = 0;
			if (!repeat || len == 129 || end) {
				/* write out repeating bytes */
				put(out, end ? len : len - 1);
				put(out, buf[0]);
				buf[0] = buf[len - 1];
				len = 1;
			}
		} else {
			if (buf[len - 1] == buf[len - 2]) {
				repeat = 1;
				if (len > 2) {
					output(out, buf, len - 2);
					buf[0] = buf[1] = buf[len - 1];
					len = 2;
				}
				continue;
			}
			if (len == 128 || end) {
				output(out, buf, len);
				len = 0;
				repeat = 0;
			}
		}
	}
	put(out, -1);
}
 
void decode(stream in, stream out) {
	int c, i, cnt;
	while (1) {
		c = in->get(in);
		if (c == -1) {
			return;
		}
		if (c > 128) {
			cnt = c - 128;
			for (i = 0; i < cnt; i++)
				out->put(out, in->get(in));
		} else {
			cnt = c;
			c = in->get(in);
			for (i = 0; i < cnt; i++)
				out->put(out, c);
		}
	}
}

}	// namespace rle
}	// namespace fileio
