/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#ifndef ARDUINO_COM_HELPERS_H
#define ARDUINO_COM_HELPERS_H



#define START_BYTE  0x02
#define END_BYTE    0x03
#define ESCAPE_BYTE 0x1B

/*!
 * A macro that defines push.
 *
 * \param   x   The character to push into an array.
 */
#define push(x) do{*d++ = x; destlen += 1;}while(0)  // C-style

/*!
 * forward declarations.
 *
 * \param [in,out]  dest    If non-null, destination for the.frame.
 * \param [in]  src     If non-null, source for the frame.
 * \param           srclen  The source length.
 *
 * \return  The length of the new frame stored in dest.
 */
int stuff_frame(void *dest, void *src, std::size_t srclen)
{
    uint8_t* d = static_cast<uint8_t*>(dest);
    uint8_t* s = static_cast<uint8_t*>(src);

    int destlen = 0;
    push(START_BYTE);
    for (std::size_t i = 0; i < srclen; i++)
    {
        if (s[i] == START_BYTE || s[i] == END_BYTE || s[i] == ESCAPE_BYTE)
        {
            push(ESCAPE_BYTE);
            push(s[i]);
        }
        else
        {
            push(s[i]);
        }
    }
    push(END_BYTE);
    return destlen;
}
#undef push

/*!
 * Destuff frame.
 *
 * \param [in,out]  dest    If non-null, destination for the. frame.
 * \param [in]  src     If non-null, source for the frame.
 * \param           srclen  The source length.
 *
 * \return  The length of the new frame.
 */
int destuff_frame(void *dest, void *src, std::size_t srclen)
{
    uint8_t* d = static_cast<uint8_t*>(dest);
    uint8_t* s = static_cast<uint8_t*>(src);

    int destlen = 0;

    for (std::size_t i = 0; i < srclen; i++)
    {
        if (s[i] == END_BYTE) break;
        if (s[i] == START_BYTE || s[i] == ESCAPE_BYTE)
        {
            *d++ = s[++i];
            destlen++;
        }
        else
        {
            *d++ = s[i];
            destlen++;
        }

    }
    return destlen;
}

/*!
 * Stuff frame.
 *
 * \param [in]  src Source for the.frame.
 *
 * \return  The stuffed frame stored in std::vector
 */
std::vector<uint8_t> stuff_frame(std::vector<uint8_t>& src)
{
    std::vector<uint8_t> dest;

    dest.push_back(START_BYTE);
    for (std::size_t i = 0; i < src.size(); i++)
    {
        if (src[i] == START_BYTE || src[i] == END_BYTE || src[i] == ESCAPE_BYTE)
        {
            dest.push_back(ESCAPE_BYTE);
            dest.push_back(src[i]);
        }
        else
        {
            dest.push_back(src[i]);
        }
    }
    dest.push_back(END_BYTE);

    return dest;
}

/*!
 * Destuff frame.
 *
 * \param [in]  src Source for the.
 *
 * \return  The destuffed frame stored in std::vector.
 */
std::vector<uint8_t> destuff_frame(std::vector<uint8_t>& src)
{
    std::vector<uint8_t> dest;

    for (std::size_t i = 0; i < src.size(); i++)
    {
        if (src[i] == END_BYTE) break;
        if (src[i] == START_BYTE || src[i] == ESCAPE_BYTE)
        {
            dest.push_back(src[++i]);
        }
        else
        {
            dest.push_back(src[i]);
        }
    }
    return dest;
}

/*!
 * Fletcher 16.
 *
 * \param   data    The data.
 * \param   bytes   The bytes.
 *
 * \return  An uint16_t.
 */
uint16_t fletcher16(uint8_t const *data, uint8_t bytes)
{

    uint16_t sum1 = 0xff, sum2 = 0xff;

    while (bytes)
    {
        uint8_t tlen = bytes > 20 ? 20 : bytes;
        bytes -= tlen;
        do
        {
            sum2 += sum1 += *data++;
        }
        while (--tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    /* Second reduction step to reduce sums to 8 bits */
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
    return sum2 << 8 | sum1;
}


#endif
