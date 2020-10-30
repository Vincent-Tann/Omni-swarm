/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __FisheyeFrameDescriptor_t_hpp__
#define __FisheyeFrameDescriptor_t_hpp__

#include <lcm/lcm_coretypes.h>

#include <vector>
#include "Time_t.hpp"
#include "ImageDescriptor_t.hpp"


class FisheyeFrameDescriptor_t
{
    public:
        int64_t    msg_id;

        int32_t    image_num;

        int8_t     prevent_adding_db;

        int32_t    landmark_num;

        int32_t    drone_id;

        Time_t     timestamp;

        std::vector< ImageDescriptor_t > images;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "FisheyeFrameDescriptor_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int FisheyeFrameDescriptor_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int FisheyeFrameDescriptor_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int FisheyeFrameDescriptor_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t FisheyeFrameDescriptor_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* FisheyeFrameDescriptor_t::getTypeName()
{
    return "FisheyeFrameDescriptor_t";
}

int FisheyeFrameDescriptor_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->msg_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->image_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->prevent_adding_db, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->landmark_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->timestamp._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->image_num; a0++) {
        tlen = this->images[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int FisheyeFrameDescriptor_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->msg_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->image_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->prevent_adding_db, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->landmark_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->drone_id, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->timestamp._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    try {
        this->images.resize(this->image_num);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->image_num; a0++) {
        tlen = this->images[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int FisheyeFrameDescriptor_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += this->timestamp._getEncodedSizeNoHash();
    for (int a0 = 0; a0 < this->image_num; a0++) {
        enc_size += this->images[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

uint64_t FisheyeFrameDescriptor_t::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == FisheyeFrameDescriptor_t::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, FisheyeFrameDescriptor_t::getHash };

    uint64_t hash = 0x1be8d9cb8f34e720LL +
         Time_t::_computeHash(&cp) +
         ImageDescriptor_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
