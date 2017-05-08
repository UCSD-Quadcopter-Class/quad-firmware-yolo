#ifndef CHECKSUM_HEADER
#define CHECKSUM_HEADER
#include <stdint.h>
#include <stdlib.h>
/**
 * Compute 8-bit checksum
 * Implementation source:
 * https://en.wikipedia.org/wiki/Fletcher%27s_checksum#Implementation
 */
uint16_t fletcher16( uint8_t const *data, size_t bytes )
{
        uint16_t sum1 = 0xff, sum2 = 0xff;
        size_t tlen;
 
        while (bytes) {
                tlen = ((bytes >= 20) ? 20 : bytes);
                bytes -= tlen;
                do {
                        sum2 += sum1 += *data++;
                        tlen--;
                } while (tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
        return (sum2 << 8) | sum1;
}
#endif
