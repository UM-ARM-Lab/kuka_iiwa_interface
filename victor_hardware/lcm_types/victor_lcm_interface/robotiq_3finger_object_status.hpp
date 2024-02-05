/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __victor_lcm_interface_robotiq_3finger_object_status_hpp__
#define __victor_lcm_interface_robotiq_3finger_object_status_hpp__


namespace victor_lcm_interface
{

class robotiq_3finger_object_status
{
    public:
        double     timestamp;

        int8_t     status;

    public:
        static constexpr int8_t   IN_MOTION = 0;
        static constexpr int8_t   AT_REQUESTED = 1;
        static constexpr int8_t   STOPPED = 2;
        static constexpr int8_t   CONTACT_OPENING = 3;
        static constexpr int8_t   CONTACT_CLOSING = 4;

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
         * @param maxlen The maximum number of bytes to reqad while decoding.
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
         * Returns "robotiq_3finger_object_status"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int robotiq_3finger_object_status::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int robotiq_3finger_object_status::decode(const void *buf, int offset, int maxlen)
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

int robotiq_3finger_object_status::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t robotiq_3finger_object_status::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* robotiq_3finger_object_status::getTypeName()
{
    return "robotiq_3finger_object_status";
}

int robotiq_3finger_object_status::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int robotiq_3finger_object_status::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int robotiq_3finger_object_status::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t robotiq_3finger_object_status::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x69f7a795f409b44fLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
