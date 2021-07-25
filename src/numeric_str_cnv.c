#include "numeric_str_cnv.h"

#define FLTERR 1.0e-6f

/** int2str converts the integer n to string padded to width using
 *  the padc character. Set width to 0 for no padding. Set pad to ' '
 *  (space) for normal right justification of string to width or use
 *  '0' to zero pad the value. (or any other character you choose)
 */
char *int2str (char *s, int n, unsigned width, unsigned char padc)
{
    char tmp[IVMAXC], *p = tmp + IVMAXC - 1;
    int i, sign = n < 0 ? 1 : 0;

    if (sign)                                       /* work with positive n */
        n = -n;

    if (width >= IVMAXC)                            /* limit width value */
        width = IVMAXC - (sign ? 2 : 1);

    *p = 0;                                         /* nul-terminate tmp */

    if (!n) {
        *--p = '0';
        if (width)
            width -= 1;
    }

    while (p > tmp && n) {                          /* convert n */
        *--p = n % 10 + '0';
        n /= 10;
        if (width)                                  /* decrement width */
            width -= 1;
    }
    if (p > tmp && sign && width)                   /* if sign, add '-' */
        *--p = '-', width--;

    while (width-- && p > tmp)                      /* pad to width */
        *--p = padc;

    for (i = 0;; i++, p++) {                        /* copy to s with \0 */
        s[i] = *p;
        if (!*p)
            break;
    }

    return s;
}

/**
 *  convert float f to string with fractional part
 *  limited to prec digits padded to width spaces.
 *  s must have adequate storage to hold the converted
 *  value. If converted value does not fit in width
 *  the string is filled with "INV" for INVALID.
 */
char *float2strw (char *s, int width, float f, int prec)
{
    int i,
        sign = f < 0 ? 1 : 0,                       /* set sign if negative */
        mult = 1,                                   /* multiplier for precision */
        cnt = 0;                                    /* digit count */
    uint32_t fpm;                                   /* floating-pointer multiplied */
    float round = .5f;                              /* set round factor */

    if (sign)                                       /* work with positive value */
        f = -f;

    s[width] = 0;                                   /* nul-terminate s */

    if (-FLTERR < f && f < FLTERR) {                /* handle zero case */
        while (width && prec--)
            s[--width] = '0';                       /* pad fp to prec with '0' */
        if (width)
            s[--width] = '.';                       /* separator */
        if (width)
            s[--width] = '0';                       /* leading 0 */
        while (width)                               /* pad to width with spaces */
            s[--width] = ' ';
        return s;
    }

    for (i = 0; i < prec; i++)                      /* compute multiplier */
        mult *= 10;

    if ((uint32_t)(f * mult) == 0)                  /* if multiple still 0 */
        round = 0.5 / (mult * 10);                  /* adjust round factor */
    fpm = (uint32_t)(f * mult + round);             /* mult entire fp, set round */

    while (width && fpm) {                          /* convert multiplied value */
        s[--width] = fpm % 10 + '0';
        fpm /= 10;
        cnt += 1;
        if (width && cnt == prec)                   /* cnt == prec, add '.' */
            s[--width] = '.';
    }
    if (width && cnt < prec) {                      /* cnt < precision */
        while (width && cnt < prec) {               /* set precision & decimal pt */
            s[--width] = '0';
            cnt += 1;
        }
        if (width)
            s[--width] = '.';
    }

    if (width && s[width] == '.')                   /* ensure leading 0 if no ip */
        s[--width] = '0';

    if (sign && width)                              /* if sign, add '-' */
        s[--width] = '-';
    else if ((sign && !width) || fpm) {             /* no room for '-' or all digits */
        s[0] = 'I';                                 /* INVALID, "INV" */
        s[1] = 'N';
        s[2] = 'V';
        s[3] = 0;
    }

    while (width)                                   /* pad to width with spaces */
        s[--width] = ' ';

    return s;
}

/**
 *  Convert string to uint_fast16_t
 */
int_fast8_t str2uint16 (char *s, uint_fast16_t *u)
{
    if (!s || !*s)
        return -1;

    for (*u = 0; *s && '0' <= *s && *s <= '9'; s++)
        *u = *u * 10 + *s - '0';

    return 1;
}

