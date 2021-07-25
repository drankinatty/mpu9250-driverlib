#ifndef __numeric_str_cnv__
#define __numeric_str_cnv__  1

#include <stdint.h>

#define FPMAXC 32                           /* floating-point max chars */
#define IVMAXC FPMAXC                       /* integer value max chars */

/* int to string */
char *int2str (char *s, int n, unsigned width, unsigned char padc);
/* float to string */
char *float2strw (char *s, int width, float f, int prec);
/* string to uint_fast16_t */
int_fast8_t str2uint16 (char *s, uint_fast16_t *u);


#endif
