#ifndef __PATCH_H
#define __PATCH_H

#include <stddef.h>
extern void *memcpy (void *__restrict __dest, const void *__restrict __src,   size_t __n);
extern int printf(const char *__restrict __format, ...);
extern int puts(const char *__s);

// Ethernet Transport Layer
#define OPTION_USE_TCP                  OFF
#define OPTION_MTU                      1500            // Ethernet MTU
#define OPTION_SERVER_PORT              5555            // Default UDP port
#define OPTION_SERVER_ADDR              {127,0,0,1}     // IP addr to bind, 0.0.0.0 = ANY


#define MAX_PATH 256
#define BOOL int
#define FALSE 0
#define TRUE 1

#endif
