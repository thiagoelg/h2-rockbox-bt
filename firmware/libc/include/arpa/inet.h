#ifndef _INET_H_
#define _INET_H_
#include "autoconf.h"
#ifndef ROCKBOX_LITTLE_ENDIAN
#define htonl(x) (x)
#define htons(x) (x)
#define ntohl(x) (x)
#define ntohs(x) (x)
#else
#include <stdint.h>

static inline uint32_t ntohl(uint32_t x)
{
	return ((((x) & 0xff000000) >> 24) | \
	        (((x) & 0x00ff0000) >>  8) | \
	        (((x) & 0x0000ff00) <<  8) | \
	        (((x) & 0x000000ff) << 24));
}

static inline uint16_t ntohs(uint16_t x)
{
	return ((((x) & 0xff00) >> 8) | \
	        (((x) & 0x00ff) << 8));
}

#define htonl(x) ntohl(x)
#define htons(x) ntohs(x)
#endif /* ROCKBOX_LITTLE_ENDIAN */
#endif /* _INET_H_ */