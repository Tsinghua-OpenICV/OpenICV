

#ifndef CANDECODER_H
#define CANDECODER_H
extern "C"
{
 
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////
// data in  hexadecimal
inline void printdata(const unsigned char * data, const unsigned long length, const int id)
{
    printf("\ndata = ");
    for (unsigned int i = 0;i< length;++i )
        printf("%02x ",data[i]);
    printf(" dlc = %ld ID = 0x%x\n", length, id);
}

//////////////////////////////////////////////////////////////////////////
// for Big Endian - Motorola coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 1 byte
// ie if length is greater than ((startBit%8) + 1)
// example:
//   7654 3210
// 0 .... ..||
// 1 |||| ....
// startBit is 1 and the length is 8: result gives (1%8)+1 = 2 < length(=6)
inline bool mSpreadOnMoreThan1Byte(const unsigned int startBit, const unsigned int length)
{
    if ( (length  > (startBit & 0x07) + 1) ) {
        return true;
    } else {
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// for Little Endian - Intel coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 1 byte
// ie if length is greater than 8-(startBit%8)
// example:
//   7654 3210
// 0 ||.. ....
// 1 .... ||||
// startBit is 6 and the length is 6: result gives 8-((6%8)) = 2 < length(=6)
inline bool iSpreadOnMoreThan1Byte(const unsigned int startBit, const unsigned int length)
{
    if ( length  > 8 - (startBit & 0x07) ) {
        return true;
    } else {
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// for Big Endian - Motorola coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 2 bytes
// ie if (length - ((startBit%8) + 1) is greater than 8
// example:
//   7654 3210
// 0 .... ...|
// 1 |||| ||||
// 2 ||.. ....
// the start bit is 0 and the length is 11: result gives 11 - (0%8+1) = 10 (>8)
//////////////////////////////////////////////////////////////////////////
inline bool mSpreadOnMoreThan2Bytes(const unsigned int startBit, const unsigned int length)
{
    if ( length - ((startBit & 0x07) + 1) > 8 )
        return true;
    else
        return false;
}

//////////////////////////////////////////////////////////////////////////
// for Little Endian - Intel coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 2 bytes
// ie if (length - (8 - startBit)) is greater than 8
// example:
//   7654 3210
// 0 |... ....
// 1 |||| ||||
// 2 .... ..||
// the start bit is 7 and the length is 11: result gives 11 - (8 - 7) = 10 (>8)
//////////////////////////////////////////////////////////////////////////
inline bool iSpreadOnMoreThan2Bytes(const unsigned int startBit, const unsigned int length)
{
    if ( length - (8 - (startBit & 0x07)) > 8 )
        return true;
    else
        return false;
}

//////////////////////////////////////////////////////////////////////////
// for Big Endian - Motorola coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 4 bytes
// ie if length is greater than ((startBit%8) + 1)
// example:
//   7654 3210
// 0 .... ..||
// 1 |||| ||||
// 2 |||| ||||
// 3 |||| ||||
// 4 |||. ....
// startBit is 1 and the length is 29: result gives 29 - ((1%8)+1) = 27 (> 24)
inline bool mSpreadOnMoreThan4Bytes(const unsigned int startBit, const unsigned int length)
{
    return (3*8) < (length - ((startBit & 0x07) + 1));
}

//////////////////////////////////////////////////////////////////////////
// for Little Endian - Intel coding
//------------------------------------------------------------------------
// return true if the signal is spread on more than 4 bytes
// ie if length is greater than 8-(startBit%8)
// example:
//   7654 3210
// 0 ||.. ....
// 1 |||| ||||
// 2 |||| ||||
// 3 |||| ||||
// 4 .... ..||
// startBit is 6 and the length is 28: result gives 28 - (8-((6%8))) = 26  (>24)
inline bool iSpreadOnMoreThan4Bytes(const unsigned int startBit, const unsigned int length)
{
    return (3*8) < (length - (8 - (startBit & 0x07)));
}

//////////////////////////////////////////////////////////////////////////
// return a 8-bits shifted-left mask corresponding to the length parameter
// ex: shift = 2 and length = 3
// mask will be 0b00011100
inline unsigned char getShiftMask8(const unsigned int shift, const unsigned int length)
{
    unsigned char mask;
    switch (length)
    {
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
    default: mask = 0; break;
    }
    return mask << shift;
}

//////////////////////////////////////////////////////////////////////////
// return a 8-bits mask corresponding to the length parameter with bit order decreasing
// ex: startBit = 2 and length = 3
// mask will be 0b00000111
inline unsigned char getMask8( const unsigned int length )
{
    unsigned char mask;
    switch (length) {
    case 1: mask = 0x01; break;
    case 2: mask = 0x03; break;
    case 3: mask = 0x07; break;
    case 4: mask = 0x0F; break;
    case 5: mask = 0x1F; break;
    case 6: mask = 0x3F; break;
    case 7: mask = 0x7F; break;
    case 8: mask = 0xFF; break;
    default: mask = 0; break;
    }
    return mask;
}

//////////////////////////////////////////////////////////////////////////
// return a 16-bits shifted-left mask corresponding to the length parameter
inline unsigned short getShiftMask16(const unsigned int shift, const unsigned int length)
{
    unsigned short mask;
    switch (length) {
    case 1: mask = 0x0001; break;
    case 2: mask = 0x0003; break;
    case 3: mask = 0x0007; break;
    case 4: mask = 0x000F; break;
    case 5: mask = 0x001F; break;
    case 6: mask = 0x003F; break;
    case 7: mask = 0x007F; break;
    case 8: mask = 0x00FF; break;
    case 9: mask = 0x01FF; break;
    case 10: mask = 0x03FF; break;
    case 11: mask = 0x07FF; break;
    case 12: mask = 0x0FFF; break;
    case 13: mask = 0x1FFF; break;
    case 14: mask = 0x3FFF; break;
    case 15: mask = 0x7FFF; break;
    case 16: mask = 0xFFFF; break;
    default: mask = 0; break;
    }
    return mask << shift;
}

//////////////////////////////////////////////////////////////////////////
// return a 16-bits mask corresponding to the length parameter with bit order decreasing
inline unsigned short getMask16( const unsigned int length )
{
    unsigned short mask;
    switch (length) {
    case 1: mask = 0x0001; break;
    case 2: mask = 0x0003; break;
    case 3: mask = 0x0007; break;
    case 4: mask = 0x000F; break;
    case 5: mask = 0x001F; break;
    case 6: mask = 0x003F; break;
    case 7: mask = 0x007F; break;
    case 8: mask = 0x00FF; break;
    case 9: mask = 0x01FF; break;
    case 10: mask = 0x03FF; break;
    case 11: mask = 0x07FF; break;
    case 12: mask = 0x0FFF; break;
    case 13: mask = 0x1FFF; break;
    case 14: mask = 0x3FFF; break;
    case 15: mask = 0x7FFF; break;
    case 16: mask = 0xFFFF; break;
    default: mask = 0; break;
    }
    return mask;
}

//////////////////////////////////////////////////////////////////////////
// return a 32-bits shifted-left mask corresponding to the length parameter
inline unsigned long getShiftMask32(const unsigned int shift, const unsigned int length)
{
    unsigned long mask;
    switch (length) {
    case 1: mask = 0x00000001; break;
    case 2: mask = 0x00000003; break;
    case 3: mask = 0x00000007; break;
    case 4: mask = 0x0000000F; break;
    case 5: mask = 0x0000001F; break;
    case 6: mask = 0x0000003F; break;
    case 7: mask = 0x0000007F; break;
    case 8: mask = 0x000000FF; break;
    case 9: mask = 0x000001FF; break;
    case 10: mask = 0x000003FF; break;
    case 11: mask = 0x000007FF; break;
    case 12: mask = 0x00000FFF; break;
    case 13: mask = 0x00001FFF; break;
    case 14: mask = 0x00003FFF; break;
    case 15: mask = 0x00007FFF; break;
    case 16: mask = 0x0000FFFF; break;
    case 17: mask = 0x0001FFFF; break;
    case 18: mask = 0x0003FFFF; break;
    case 19: mask = 0x0007FFFF; break;
    case 20: mask = 0x000FFFFF; break;
    case 21: mask = 0x001FFFFF; break;
    case 22: mask = 0x003FFFFF; break;
    case 23: mask = 0x007FFFFF; break;
    case 24: mask = 0x00FFFFFF; break;
    case 25: mask = 0x01FFFFFF; break;
    case 26: mask = 0x03FFFFFF; break;
    case 27: mask = 0x07FFFFFF; break;
    case 28: mask = 0x0FFFFFFF; break;
    case 29: mask = 0x1FFFFFFF; break;
    case 30: mask = 0x3FFFFFFF; break;
    case 31: mask = 0x7FFFFFFF; break;
    case 32: mask = 0xFFFFFFFF; break;
    default: mask = 0; break;
    }
    return mask << shift;
}

//////////////////////////////////////////////////////////////////////////
// return a 32-bits mask corresponding to the length parameter with bit order decreasing
inline unsigned long getMask32( const unsigned int length )
{
    unsigned long mask;
    switch (length) {
    case 1: mask = 0x00000001; break;
    case 2: mask = 0x00000003; break;
    case 3: mask = 0x00000007; break;
    case 4: mask = 0x0000000F; break;
    case 5: mask = 0x0000001F; break;
    case 6: mask = 0x0000003F; break;
    case 7: mask = 0x0000007F; break;
    case 8: mask = 0x000000FF; break;
    case 9: mask = 0x000001FF; break;
    case 10: mask = 0x000003FF; break;
    case 11: mask = 0x000007FF; break;
    case 12: mask = 0x00000FFF; break;
    case 13: mask = 0x00001FFF; break;
    case 14: mask = 0x00003FFF; break;
    case 15: mask = 0x00007FFF; break;
    case 16: mask = 0x0000FFFF; break;
    case 17: mask = 0x0001FFFF; break;
    case 18: mask = 0x0003FFFF; break;
    case 19: mask = 0x0007FFFF; break;
    case 20: mask = 0x000FFFFF; break;
    case 21: mask = 0x001FFFFF; break;
    case 22: mask = 0x003FFFFF; break;
    case 23: mask = 0x007FFFFF; break;
    case 24: mask = 0x00FFFFFF; break;
    case 25: mask = 0x01FFFFFF; break;
    case 26: mask = 0x03FFFFFF; break;
    case 27: mask = 0x07FFFFFF; break;
    case 28: mask = 0x0FFFFFFF; break;
    case 29: mask = 0x1FFFFFFF; break;
    case 30: mask = 0x3FFFFFFF; break;
    case 31: mask = 0x7FFFFFFF; break;
    case 32: mask = 0xFFFFFFFF; break;
    default: mask = 0; break;
    }
    return mask;
}

//////////////////////////////////////////////////////////////////////////
// return a 64-bits mask corresponding to the length parameter with bit order decreasing
inline unsigned long long getMask64( const unsigned int length )
{
    unsigned long long mask;
    switch (length) {
    case 1: mask = 0x0000000000000001ULL; break;
    case 2: mask = 0x0000000000000003ULL; break;
    case 3: mask = 0x0000000000000007ULL; break;
    case 4: mask = 0x000000000000000FULL; break;
    case 5: mask = 0x000000000000001FULL; break;
    case 6: mask = 0x000000000000003FULL; break;
    case 7: mask = 0x000000000000007FULL; break;
    case 8: mask = 0x00000000000000FFULL; break;
    case 9: mask = 0x00000000000001FFULL; break;
    case 10: mask = 0x00000000000003FFULL; break;
    case 11: mask = 0x00000000000007FFULL; break;
    case 12: mask = 0x0000000000000FFFULL; break;
    case 13: mask = 0x0000000000001FFFULL; break;
    case 14: mask = 0x0000000000003FFFULL; break;
    case 15: mask = 0x0000000000007FFFULL; break;
    case 16: mask = 0x000000000000FFFFULL; break;
    case 17: mask = 0x000000000001FFFFULL; break;
    case 18: mask = 0x000000000003FFFFULL; break;
    case 19: mask = 0x000000000007FFFFULL; break;
    case 20: mask = 0x00000000000FFFFFULL; break;
    case 21: mask = 0x00000000001FFFFFULL; break;
    case 22: mask = 0x00000000003FFFFFULL; break;
    case 23: mask = 0x00000000007FFFFFULL; break;
    case 24: mask = 0x0000000000FFFFFFULL; break;
    case 25: mask = 0x0000000001FFFFFFULL; break;
    case 26: mask = 0x0000000003FFFFFFULL; break;
    case 27: mask = 0x0000000007FFFFFFULL; break;
    case 28: mask = 0x000000000FFFFFFFULL; break;
    case 29: mask = 0x000000001FFFFFFFULL; break;
    case 30: mask = 0x000000003FFFFFFFULL; break;
    case 31: mask = 0x000000007FFFFFFFULL; break;
    case 32: mask = 0x00000000FFFFFFFFULL; break;
    case 33: mask = 0x00000001FFFFFFFFULL; break;
    case 34: mask = 0x00000003FFFFFFFFULL; break;
    case 35: mask = 0x00000007FFFFFFFFULL; break;
    case 36: mask = 0x0000000FFFFFFFFFULL; break;
    case 37: mask = 0x0000001FFFFFFFFFULL; break;
    case 38: mask = 0x0000003FFFFFFFFFULL; break;
    case 39: mask = 0x0000007FFFFFFFFFULL; break;
    case 40: mask = 0x000000FFFFFFFFFFULL; break;
    case 41: mask = 0x000001FFFFFFFFFFULL; break;
    case 42: mask = 0x000003FFFFFFFFFFULL; break;
    case 43: mask = 0x000007FFFFFFFFFFULL; break;
    case 44: mask = 0x00000FFFFFFFFFFFULL; break;
    case 45: mask = 0x00001FFFFFFFFFFFULL; break;
    case 46: mask = 0x00003FFFFFFFFFFFULL; break;
    case 47: mask = 0x00007FFFFFFFFFFFULL; break;
    case 48: mask = 0x0000FFFFFFFFFFFFULL; break;
    case 49: mask = 0x0001FFFFFFFFFFFFULL; break;
    case 50: mask = 0x0003FFFFFFFFFFFFULL; break;
    case 51: mask = 0x0007FFFFFFFFFFFFULL; break;
    case 52: mask = 0x000FFFFFFFFFFFFFULL; break;
    case 53: mask = 0x001FFFFFFFFFFFFFULL; break;
    case 54: mask = 0x003FFFFFFFFFFFFFULL; break;
    case 55: mask = 0x007FFFFFFFFFFFFFULL; break;
    case 56: mask = 0x00FFFFFFFFFFFFFFULL; break;
    case 57: mask = 0x01FFFFFFFFFFFFFFULL; break;
    case 58: mask = 0x03FFFFFFFFFFFFFFULL; break;
    case 59: mask = 0x07FFFFFFFFFFFFFFULL; break;
    case 60: mask = 0x0FFFFFFFFFFFFFFFULL; break;
    case 61: mask = 0x1FFFFFFFFFFFFFFFULL; break;
    case 62: mask = 0x3FFFFFFFFFFFFFFFULL; break;
    case 63: mask = 0x7FFFFFFFFFFFFFFFULL; break;
    case 64: mask = 0xFFFFFFFFFFFFFFFFULL; break;
    default: mask = 0; break;
    }
    return mask;
}

//////////////////////////////////////////////////////////////////////////
/// Returns a 64-bits shifted-left mask corresponding to the length parameter
inline unsigned long long getShiftMask64(const unsigned int shift, const unsigned int length)
{
    unsigned long long mask;
    mask = getMask64( length );
    mask <<= shift;
    return mask;
}

//////////////////////////////////////////////////////////////////////////
/// Big Endian - Motorola coding
inline bool mDecodeToBool(const unsigned char * data, const unsigned int startBit)
{
    return (data[startBit>>3] & getShiftMask8(startBit&0x07,1)) != 0;
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// to test ?
inline bool mDecodeToUI64(unsigned long long * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 64) {
        *result = 0;
        return false;
    }

    if (length == 64) {
        // only work if data are correctly byte-aligned
        unsigned char c[8];   // c[0] = MSB
        for (int i = 0 ; i < 8 ; i++)
            c[i] = data[(startBit>>3) + i];

        unsigned long long temp = 0;
        for (int i = 0 ; i < 8 ; i++)
            temp += (c[i] << ( 56 - (8*i) ) );
        *result = temp;
        return true;
    } else {
        unsigned char c[8], mask[8], nbBits[8]; // MSB = c[0]
        nbBits[0] = (startBit & 0x07)+1;
        mask[0] = getMask8( nbBits[0] );
        c[0] = data[startBit>>3] & mask[0];
        unsigned short nbBitsIncrement = nbBits[0];
        for (int i = 1 ; i < 8 ; i++) {
            nbBits[i] = static_cast<unsigned char>( (length - nbBitsIncrement) < 8 ? length - nbBitsIncrement : 8 );
            nbBitsIncrement += nbBits[i];
            mask[i] = getShiftMask8(8 - nbBits[i],nbBits[i]);
            c[i] = data[(startBit>>3) + i] & mask[i];
        }

        unsigned long long temp = 0;
        for (int i = 0 ; i < 8 ; i++)
            temp += (c[i] << ( 56 - (8*i) ) );
        *result = temp >> (56 + (startBit&0x07) + 1 - length);
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// to test ?
inline bool mDecodeToI64(long long *result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>64) {
        *result = 0;
        return false;
    }

    if (length == 64) {
        // only work if data are correctly byte-aligned
        unsigned char c[8];   // c[0] = MSB
        for (int i = 0 ; i < 8 ; i++)
            c[i] = data[(startBit>>3) + i];

        for (int i = 0 ; i < 8 ; i++)
            *result += (c[i] << ( 56 - (8*i) ) );

        // need to be signed ??
        return true;
    } else {
        unsigned char c[8], mask[8], nbBits[8]; // MSB = c[0]
        nbBits[0] = (startBit & 0x07)+1;
        mask[0] = getMask8( nbBits[0] );
        c[0] = data[startBit>>3] & mask[0];
        unsigned short nbBitsIncrement = nbBits[0];
        for (int i = 1 ; i < 8 ; i++) {
            nbBits[i] = static_cast<unsigned char>( (length - nbBitsIncrement) < 8 ? (length - nbBitsIncrement) : 8 );
            nbBitsIncrement += nbBits[i];
            mask[i] = getShiftMask8(8 - nbBits[i],nbBits[i]);
            c[i] = data[(startBit>>3) + i] & mask[i];
        }
        unsigned long long temp = 0;
        for (int i = 0 ; i < 8 ; i++)
            temp += (c[i] << ( 56 - (8*i) ) );
        temp >>= (56 + (startBit&0x07) + 1 - length);

        if ( temp & getShiftMask64(length-1,1) )
            // do the two's complement to get the signed value if the msb=1
            *result = -1 * ( ( (~temp) + 1 ) & getMask64(length) );
        else
            *result = temp;

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mDecodeToUI32(unsigned long *result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>32) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 5 bytes
    if (mSpreadOnMoreThan4Bytes(startBit, length)) {
        // decode in a 64-bits integer
        unsigned long long temp;
        if (mDecodeToUI64(&temp,data,startBit,length))
        {
            *result = static_cast<unsigned long> (temp);
            return true;
        } else {
            *result = 0;
            return false;
        }
    } else {
        if (length == 32) {
            // only work if data are correctly byte-aligned
            unsigned char c1, c2, c3, c4;
            c1 = data[startBit>>3];
            c2 = data[(startBit>>3) + 1];
            c3 = data[(startBit>>3) + 2];
            c4 = data[(startBit>>3) + 3];

            *result = (c1 << 24) + (c2 << 16) + (c3 << 8) + c4;
            return true;
        } else {
            unsigned char c[4], mask[4], nbBits[4]; // MSB = c[0]
            nbBits[0] = (startBit & 0x07)+1;
            nbBits[1] = static_cast<unsigned char>( (length - nbBits[0]) < 8 ? length - nbBits[0] : 8 );
            nbBits[2] = static_cast<unsigned char>( (length - nbBits[0] - nbBits[1]) < 8 ? (length - nbBits[0] - nbBits[1]) : 8 );
            nbBits[3] = static_cast<unsigned char>( (length - nbBits[0] - nbBits[1] - nbBits[2]) < 8 ? (length - nbBits[0] - nbBits[1] - nbBits[2]) : 8 );
            mask[0] = getMask8( nbBits[0] );
            mask[1] = getShiftMask8(8 - nbBits[1],nbBits[1]);
            mask[2] = getShiftMask8(8 - nbBits[2],nbBits[2]);
            mask[3] = getShiftMask8(8 - nbBits[3],nbBits[3]);
            c[0] = data[startBit>>3] & mask[0];
            c[1] = data[(startBit>>3) + 1] & mask[1];
            c[2] = data[(startBit>>3) + 2] & mask[2];
            c[3] = data[(startBit>>3) + 3] & mask[3];
            *result = ( (c[0]<<24) + (c[1]<<16) + (c[2]<<8) + c[3]) >> (24 + (startBit&0x07) + 1 - length);
            return true;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mDecodeToI32(long *result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>32) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 5 bytes
    if (mSpreadOnMoreThan4Bytes(startBit, length)) {
        // decode in a 64-bits integer
        long long temp;
        if (mDecodeToI64(&temp,data,startBit,length))
        {
            *result = static_cast<long>(temp);
            return true;
        } else {
            *result = 0;
            return false;
        }
    } else {
        if (length == 32)
        {
            // only work if data are correctly byte-aligned
            unsigned char c1, c2, c3, c4;
            c1 = data[startBit>>3];
            c2 = data[(startBit>>3) + 1];
            c3 = data[(startBit>>3) + 2];
            c4 = data[(startBit>>3) + 3];

            *result = (c1 << 24) + (c2 << 16) + (c3 << 8) + c4;
            return true;
        } else {
            unsigned char c[4], mask[4], nbBits[4]; // MSB = c[0]
            nbBits[0] = (startBit & 0x07) + 1;
            nbBits[1] = static_cast<unsigned char>( (length - nbBits[0]) < 8 ? length - nbBits[0] : 8 );
            nbBits[2] = static_cast<unsigned char>( (length - nbBits[0] - nbBits[1]) < 8 ? (length - nbBits[0] - nbBits[1]) : 8 );
            nbBits[3] = static_cast<unsigned char>( (length - nbBits[0] - nbBits[1] - nbBits[2]) < 8 ? length - nbBits[0] - nbBits[1] - nbBits[2] : 8 );
            mask[0] = getMask8( nbBits[0] );
            mask[1] = getShiftMask8(8 - nbBits[1],nbBits[1]);
            mask[2] = getShiftMask8(8 - nbBits[2],nbBits[2]);
            mask[3] = getShiftMask8(8 - nbBits[3],nbBits[3]);
            c[0] = data[startBit>>3] & mask[0];
            c[1] = data[(startBit>>3) + 1] & mask[1];
            c[2] = data[(startBit>>3) + 2] & mask[2];
            c[3] = data[(startBit>>3) + 3] & mask[3];

            unsigned long temp = ( (c[0]<<24) + (c[1]<<16) + (c[2]<<8) + c[3]) >> (24 + (startBit&0x07) + 1 - length);
            if (temp & getShiftMask32(length-1, 1))
                // do the two's complement to get the signed value if the msb=1
                *result = -1 * ( ( (~temp) + 1 ) & getMask32(length) );
            else
                *result = temp;
            return true;
        }
    }
}



//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mDecodeToUI16(unsigned short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 16) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 3 bytes
    if (mSpreadOnMoreThan2Bytes(startBit, length)) {
        // decode in a 32-bits integer
        unsigned long temp;
        if (mDecodeToUI32(&temp,data,startBit,length)) {
            *result = static_cast<unsigned short>(temp);
            return true;
        } else {
            *result = 0;
            return false;
        }
    } else {
        if (length == 16) {
            // only work if data are correctly byte-aligned
            unsigned char LSB, MSB;
            MSB = data[startBit>>3];
            LSB = data[(startBit>>3) + 1];

            *result = ( (MSB << 8) + LSB );
            return true;
        } else {
            unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;
            nbBitsMSB = (startBit & 0x07) + 1;  // the number of bits contained in the MSB
            nbBitsLSB = static_cast<unsigned char>(length - nbBitsMSB);
            maskMSB = getMask8( nbBitsMSB );
            maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
            MSB = data[startBit>>3] & maskMSB;
            LSB = data[(startBit>>3) + 1] & maskLSB;

            *result = ( (MSB << 8) + LSB ) >> (8 + (startBit&0x07) + 1 - length);
            return true;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mDecodeToI16(short *result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 16) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 3 bytes
    if (mSpreadOnMoreThan2Bytes(startBit, length)) {
        // decode in a 32-bits integer
        long temp;
        if (mDecodeToI32(&temp, data, startBit, length)) {
            *result = (short)temp;
            return true;
        } else  {
            *result = 0;
            return false;
        }
    } else {
        // ok data are stored at most in 2 bytes
        if (length == 16) {
            // only work if data are correctly byte-aligned
            unsigned char LSB, MSB;
            MSB = data[startBit>>3];
            LSB = data[(startBit>>3) + 1];
            short temp = (MSB << 8) + LSB;

            // Attention : a-t on besoin d'appliquer le signe ?
            // n'est-il pas deja inclu dans la donn�e comme elle est correctement align�e sur l'octet ?
            if (temp & 0x8000) {
                // do the two's complement to get the signed value
                *result = - ( ( (~temp) + 1 ) & 0xFFFF );
            } else {
                *result = temp;
            }
            return true;
        } else {
            unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;
            nbBitsMSB = (startBit & 0x07) + 1;  // the number of bits contained in the MSB
            nbBitsLSB = static_cast<unsigned char>(length - nbBitsMSB);
            maskMSB = getMask8( nbBitsMSB );
            maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
            MSB = data[startBit>>3] & maskMSB;
            LSB = data[(startBit>>3) + 1] & maskLSB;
            // assign the MSB and LSB char in the short integer value and right-shift
            // to place the lsb to the bit 0
            unsigned short temp = ( (MSB << 8) + LSB ) >> (8 + (startBit&0x07) + 1 - length);
            if (temp & getShiftMask16(length-1,1))
                // do the two's complement to get the signed value if the msb=1
                *result = -1 * ( ( (~temp) + 1 ) & getMask16(length) );
            else
                *result = temp;
            return true;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// OK
inline bool mDecodeToUI8(unsigned char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 8) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 2 bytes
    if (mSpreadOnMoreThan1Byte(startBit, length)) {
        // decode in a 16-bit integer
        unsigned short temp;
        if ( mDecodeToUI16(&temp, data, startBit, length) ) {
            *result = static_cast<unsigned char>(temp);
            return true;
        } else {
            *result = 0;
            return false;
        }
    } else {
        // ok data is stored at most in 1 byte
        unsigned char c;
        
		c = data[startBit>>3];                // >>3 <=> div 8
        //c >>= (8-((startBit & 0x07)+1));      // &0x07 <=> modulo 8
		c >>= (startBit & 0x07) - (length - 1);    // &0x07 <=> modulo 8
		*result = c & getMask8( length );

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// OK
inline bool mDecodeToI8(char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 8) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 2 bytes
    if (mSpreadOnMoreThan1Byte(startBit, length)) {
        // decode in a 16-bit integer
        short temp;
        if ( mDecodeToI16(&temp, data, startBit, length) ) {
            *result = static_cast<char>(temp);
            return true;
        } else {
            *result = 0;
            return false;
        }
    }
    else {
        // ok data is stored at most in 1 byte
        char c;
        c = data[startBit>>3];
        c >>= (8-((startBit & 0x07)+1));
        c = c & getMask8( length );

        // a-t-on besoin d'appliquer le signe dans le cas ou length = 8
        // la donnee est correctement alignee sur l'octet donc le signe est peut-etre compris
        if (c & getShiftMask8(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = - ( ( (~c) + 1 ) & getMask8(length) );
        else
            *result = c;
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// ok
inline bool iDecodeToBool(const unsigned char * data, const unsigned int startBit)
{
    mDecodeToBool(data, startBit);
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// pas de depassement possible - la trame CAN fait 64 bits max
// gerer le signe
inline bool iDecodeToUI64(unsigned long long * /*result*/, const unsigned char * /*data*/, const unsigned int /*startBit*/, const unsigned int /*length*/)
{
    return true;
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// pas de depassement possible - la trame CAN fait 64 bits max
// gerer le signe
inline bool iDecodeToI64(long long * /*result*/, const unsigned char * /*data*/, const unsigned int /*startBit*/, const unsigned int /*length*/)
{
    return true;
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// gerer le depassement sur plus de 4 octets : necessite la fonction de decodage 64 bits
// verifier le depassement avant cette ligne :   if (length == 32)
inline bool iDecodeToUI32(unsigned long * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>32) {
        *result = 0;
        return false;
    }

    if (length == 32) {
        // only work if length == 32
        unsigned char c1, c2, c3, c4;
        c4 = data[startBit>>3];       // LSB
        c3 = data[(startBit>>3) + 1];
        c2 = data[(startBit>>3) + 2];
        c1 = data[(startBit>>3) + 3]; // MSB

        *result = (c1 << 24) + (c2 << 16) + (c3 << 8) + c4;
        return true;
    } else {
        // todo
        *result = 0;
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// gerer le signe
// gerer le depassement sur plus de 4 octets : necessite la fonction de decodage 64 bits
// verifier le depassement avant cette ligne :   if (length == 32)
inline bool iDecodeToI32(long * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>32) {
        *result = 0;
        return false;
    }

    if (length == 32) {
        // only work if length == 32
        unsigned char c1, c2, c3, c4;
        c4 = data[startBit>>3];       // LSB
        c3 = data[(startBit>>3) + 1];
        c2 = data[(startBit>>3) + 2];
        c1 = data[(startBit>>3) + 3]; // MSB

        *result = (c1 << 24) + (c2 << 16) + (c3 << 8) + c4;
        return true;
    } else {
        // todo
        *result = 0;
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// gerer le depassement sur plus de 2 octets
// verifier le depassement avant cette ligne :   if (length == 16)
inline bool iDecodeToUI16(unsigned short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 16) {
        *result = 0;
        return false;
    }

    if (length == 16) {
        // only work if length == 16
        unsigned char LSB, MSB;
        LSB = data[startBit>>3];
        MSB = data[(startBit>>3) + 1];

        *result = (MSB << 8) + LSB;
        return true;
    } else {
        // TODO
        *result = 0;
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// gerer le signe
// gerer le depassement sur plus de 2 octets
// verifier le depassement avant cette ligne :   if (length == 16)
// manque le decalage
inline bool iDecodeToI16(short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 16) {
        *result = 0;
        return false;
    }

    if (length == 16) {
        // only work if length == 16
        unsigned char LSB, MSB;
        LSB = data[startBit>>3];
        MSB = data[(startBit>>3) + 1];

        *result = (MSB << 8) + LSB;
        return true;
    } else {
        // todo
        *result = 0;
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// OK
inline bool iDecodeToUI8(unsigned char * result,const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 8) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 2 bytes
    if (iSpreadOnMoreThan1Byte(startBit, length)) {
        // decode in a 16-bit integer
        unsigned short temp;
        if ( iDecodeToUI16(&temp, data, startBit, length) ) {
            // and cast in an 8 bit integer
            *result = (unsigned char) temp;
            return true;
        } else {
            *result = 0;
            return true;
        }
    } else {
        // ok data is stored at most in 1 byte
        unsigned char c;
        c = data[startBit>>3];
        c >>= (startBit & 0x07);
        *result = c & getMask8( length );

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// OK
inline bool iDecodeToI8(char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 8) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 2 bytes
    if (iSpreadOnMoreThan1Byte(startBit, length)) {
        // decode in a 16-bit integer
        short temp;
        if ( iDecodeToI16(&temp, data, startBit, length) ) {
            // and cast in an 8 bit integer
            *result = static_cast<char>(temp);
            return true;
        } else {
            *result = 0;
            return true;
        }
    } else {
        // ok data is stored at most in 1 byte
        char c;
        c = data[startBit>>3];     // >>3 <=> div 8
        c >>= (startBit & 0x07);    // &0x07 <=> modulo 8
        c = c & getMask8( length );
        if (c & getShiftMask8(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = - ( ( (~c) + 1 ) & getMask8(length) );
        else
            *result = c;

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mobileyemDecodeToUI16(unsigned short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>16){ *result = 0; return false; }

    // verify that the frame is not spread in 3 bytes
    /*    if (mSpreadOnMoreThan2Bytes(startBit, length))
    {
    // decode in a 32-bits integer
    unsigned long temp;
    if (mDecodeToUI32(&temp,data,startBit,length))
    {
    *result = static_cast<unsigned short>(temp);
    return true;
    } else {
    *result = 0;
    return false;
    }
    }
    else
    {*/
    if (length == 16) {
        // only work if data are correctly byte-aligned
        unsigned char LSB, MSB;
        MSB = data[startBit>>3];
        LSB = data[(startBit>>3) + 1];

        *result = ( (MSB << 8) + LSB );
        return true;
    } else {
        unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;
        nbBitsMSB = (startBit & 0x07) + 1;  // the number of bits contained in the MSB
        nbBitsLSB = static_cast<unsigned char>(length - nbBitsMSB);
        maskMSB = getMask8( nbBitsMSB );
        maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
        MSB = data[startBit>>3] & maskMSB;
        LSB = data[(startBit>>3) + 1] & maskLSB;

        *result = ( (MSB << 8) + LSB ) >> (8 + (startBit&0x07) + 1 - length);
        return true;
    }
    //}
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// ok
inline bool mobileyemDecodeToI16(short *result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>16) { *result = 0; return false; }

    // verify that the frame is not spread in 3 bytes
    /*    if (mSpreadOnMoreThan2Bytes(startBit, length))
    {
    // decode in a 32-bits integer
    long temp;
    if (mDecodeToI32(&temp, data, startBit, length)) {
    *result = (short)temp;
    return true;
    } else  {
    *result = 0;
    return false;
    }
    }
    else  // ok data are stored at most in 2 bytes
    {*/
    if (length == 16)
    {
        // only work if data are correctly byte-aligned
        unsigned char LSB, MSB;
        MSB = data[startBit>>3];
        LSB = data[(startBit>>3) + 1];
        short temp = (MSB << 8) + LSB;

        // Attention : a-t on besoin d'appliquer le signe ?
        // n'est-il pas deja inclu dans la donn�e comme elle est correctement align�e sur l'octet ?
        if (temp & 0x8000) {
            // do the two's complement to get the signed value
            *result = - ( ( (~temp) + 1 ) & 0xFFFF );
        } else {
            *result = temp;
        }
        return true;
    } else {
        unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;
        nbBitsMSB = (startBit & 0x07)+1;  // the number of bits contained in the MSB
        nbBitsLSB = static_cast<unsigned char>(length - nbBitsMSB);
        maskMSB = getMask8( nbBitsMSB );
        maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
        MSB = data[startBit>>3] & maskMSB;
        LSB = data[(startBit>>3) + 1] & maskLSB;
        // assign the MSB and LSB char in the short integer value and right-shift
        // to place the lsb to the bit 0
        unsigned short temp = ( (MSB << 8) + LSB ) >> (8 + (startBit&0x07) + 1 - length);
        if (temp & getShiftMask16(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = -1 * ( ( (~temp) + 1 ) & getMask16(length) );
        else
            *result = temp;
        return true;
    }
    //}
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// OK
inline bool mobileyemDecodeToUI8(unsigned char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>8) { *result = 0; return false; }

    // verify that the frame is not spread in 2 bytes
    if (mSpreadOnMoreThan1Byte(startBit, length))
    {
        // decode in a 16-bit integer
        unsigned short temp;
        if ( mDecodeToUI16(&temp, data, startBit, length) ) {
            *result = static_cast<unsigned char>(temp);
            return true;
        } else {
            *result = 0;
            return true;
        }
    }
    else // ok data is stored at most in 1 byte
    {
        unsigned char c;
        c = data[startBit>>3];                // >>3 <=> div 8
        c >>= (8-((startBit & 0x07)+1));      // &0x07 <=> modulo 8
        *result = c & getMask8( length );

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Big Endian - Motorola coding
// OK
inline bool mobileyemDecodeToI8(char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>8) { *result = 0; return false; }

    // verify that the frame is not spread in 2 bytes
    if (mSpreadOnMoreThan1Byte(startBit, length))
    {
        // decode in a 16-bit integer
        short temp;
        if ( mDecodeToI16(&temp, data, startBit, length) ) {
            *result = static_cast<char>(temp);
            return true;
        } else {
            *result = 0;
            return true;
        }
    }
    else // ok data is stored at most in 1 byte
    {
        char c;
        c = data[startBit>>3];
        c >>= (8-((startBit & 0x07)+1));
        c = c & getMask8( length );

        // a-t-on besoin d'appliquer le signe dans le cas ou length = 8
        // la donnee est correctement alignee sur l'octet donc le signe est peut-etre compris
        if (c & getShiftMask8(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = - ( ( (~c) + 1 ) & getMask8(length) );
        else
            *result = c;
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// OK, mais gerer le depassement sur plus de 2 octets
inline bool mobileyeiDecodeToUI16(unsigned short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>16) {
        *result = 0;
        return false;
    }

    if (length == 16) {
        // only work if length == 16
        unsigned char LSB, MSB;
        LSB = data[startBit>>3];
        MSB = data[(startBit>>3) + 1];

        *result = (unsigned short)((MSB << 8) + LSB);
        return true;
    } else {
        unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;

        nbBitsLSB = static_cast<unsigned char>(8*((startBit>>3) + 1) - startBit);
        nbBitsMSB = static_cast<unsigned char>(length - nbBitsLSB);
        maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
        maskMSB = getMask8( nbBitsMSB );
        LSB = data[startBit>>3] & maskLSB;
        MSB = data[(startBit>>3) + 1] & maskMSB;

        *result = (unsigned short)((MSB<<8) + (LSB>>(startBit-8*(startBit>>3))));

        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// gerer le signe
// gerer le depassement sur plus de 2 octets
// verifier le depassement avant cette ligne :   if (length == 16)
// manque le decalage
inline bool mobileyeiDecodeToI16(short * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 16) {
        *result = 0;
        return false;
    }

    if (length == 16) {
        // only work if length == 16
        unsigned char LSB, MSB;
        LSB = data[startBit>>3];
        MSB = data[(startBit>>3) + 1];

        short temp = (MSB << 8) + LSB;

        if (temp & 0x8000) {
            // do the two's complement to get the signed value
            *result = - ( ( (~temp) + 1 ) & 0xFFFF );
        } else {
            *result = temp;
        }
        return true;
    } else {
        unsigned char LSB, MSB, maskMSB, maskLSB, nbBitsMSB, nbBitsLSB;

        nbBitsLSB = static_cast<unsigned char>(8*((startBit>>3) + 1) - startBit);
        nbBitsMSB = static_cast<unsigned char>(length - nbBitsLSB);
        maskLSB = getShiftMask8(8 - nbBitsLSB , nbBitsLSB);
        maskMSB = getMask8( nbBitsMSB );
        LSB = data[startBit>>3] & maskLSB;
        MSB = data[(startBit>>3) + 1] & maskMSB;

        unsigned short temp = (unsigned short)((MSB<<8) + (LSB>>(startBit-8*(startBit>>3))));

        if (temp & getShiftMask16(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = -1 * ( ( (~temp) + 1 ) & getMask16(length) );
        else
            *result = temp;
        return true;
    }
}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// OK
inline bool mobileyeiDecodeToUI8(unsigned char * result,const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length>8) { *result = 0; return false; }

    // verify that the frame is not spread in 2 bytes
    if (iSpreadOnMoreThan1Byte(startBit, length))
    {
        // decode in a 16-bit integer
        unsigned short temp;
        if ( iDecodeToUI16(&temp, data, startBit, length) )
        {
            // and cast in an 8 bit integer
            *result = (unsigned char) temp;
            return true;
        }
        else
        {
            *result = 0;
            return true;
        }
    }
    else // ok data is stored at most in 1 byte
    {
        unsigned char c;
        c = data[startBit>>3];
        c >>= (startBit & 0x07);
        *result = c & getMask8( length );

        return true;
    }

}

//////////////////////////////////////////////////////////////////////////
// Little Endian - Intel coding
// OK
inline bool mobileyeiDecodeToI8(char * result, const unsigned char * data, const unsigned int startBit, const unsigned int length)
{
    if (length > 8) {
        *result = 0;
        return false;
    }

    // verify that the frame is not spread in 2 bytes
    if (iSpreadOnMoreThan1Byte(startBit, length)) {
        // decode in a 16-bit integer
        short temp;
        if ( iDecodeToI16(&temp, data, startBit, length) ) {
            // and cast in an 8 bit integer
            *result = (char)(temp);
            return true;
        } else {
            *result = 0;
            return true;
        }
    } else {
        // ok data is stored at most in 1 byte
        char c;
        c = data[startBit>>3];     // >>3 <=> div 8
        c >>= (startBit & 0x07);    // &0x07 <=> modulo 8
        c = c & getMask8( length );
        if (c & getShiftMask8(length-1,1))
            // do the two's complement to get the signed value if the msb=1
            *result = - ( ( (~c) + 1 ) & getMask8(length) );
        else
            *result = c;

        return true;
    }
}

inline int mobileyeDecodeCAN(unsigned char *data,unsigned char bigEndian,unsigned int startBit,unsigned int length,unsigned char Signed)
{
    int value;
    unsigned short ustemp16 = 0;
    unsigned char uctemp8 = 0;
    short stemp16 = 0;
    char ctemp8 = 0;

    if(bigEndian) {
        if(length>8){
            if(Signed){mobileyemDecodeToI16(&stemp16,data, startBit, length);value = stemp16;}
            else {mobileyemDecodeToUI16(&ustemp16,data, startBit, length);value = ustemp16;}}
        else if(Signed) {mobileyemDecodeToI8(&ctemp8,data, startBit, length);value = ctemp8;}
        else {mobileyemDecodeToUI8(&uctemp8,data, startBit, length);value = uctemp8;}
    } else {
        if(length>8){
            if(Signed) {mobileyeiDecodeToI16(&stemp16,data, startBit, length);value = stemp16;}
            else {mobileyeiDecodeToUI16(&ustemp16,data, startBit, length);value = ustemp16;}}
        else if(Signed) {mobileyeiDecodeToI8(&ctemp8,data, startBit, length);value = ctemp8;}
        else {mobileyeiDecodeToUI8(&uctemp8,data, startBit, length);value = uctemp8;}
    }

    return value;
}
 }

#endif 
