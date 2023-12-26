#pragma once
// MESSAGE GENERAL_PARAM_REQUEST_LIST PACKING

#define TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST 4100

TTAPACKED(
typedef struct __ttalink_general_param_request_list_t {
 int64_t sn; /*< 设备序列号.*/
 uint8_t device_type; /*< 设备类型.*/
 uint32_t reserved; /*< This is reserved*/
}) ttalink_general_param_request_list_t;

#define TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN 13
#define TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN 13
#define TTALINK_MSG_ID_4100_LEN 13
#define TTALINK_MSG_ID_4100_MIN_LEN 13

#define TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC 206
#define TTALINK_MSG_ID_4100_CRC 206



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_GENERAL_PARAM_REQUEST_LIST { \
    4100, \
    "GENERAL_PARAM_REQUEST_LIST", \
    3, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_param_request_list_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_param_request_list_t, device_type) }, \
         { "reserved", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_general_param_request_list_t, reserved) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_GENERAL_PARAM_REQUEST_LIST { \
    "GENERAL_PARAM_REQUEST_LIST", \
    3, \
    {  { "sn", NULL, TTALINK_TYPE_INT64_T, 0, 0, offsetof(ttalink_general_param_request_list_t, sn) }, \
         { "device_type", NULL, TTALINK_TYPE_UINT8_T, 0, 8, offsetof(ttalink_general_param_request_list_t, device_type) }, \
         { "reserved", NULL, TTALINK_TYPE_UINT32_T, 0, 9, offsetof(ttalink_general_param_request_list_t, reserved) }, \
         } \
}
#endif


static inline uint16_t _ttalink_general_param_request_list_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, uint32_t reserved, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, reserved);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN);
#else
    ttalink_general_param_request_list_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.reserved = reserved;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
}

/**
 * @brief Pack a general_param_request_list message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_request_list_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, uint32_t reserved)
{
    return _ttalink_general_param_request_list_pack(dst_addr, src_addr, msg,  sn, device_type, reserved, false);
}

/**
 * @brief Pack a general_param_request_list message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_request_list_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               int64_t sn, uint8_t device_type, uint32_t reserved)
{
    return _ttalink_general_param_request_list_pack(dst_addr, src_addr, msg,  sn, device_type, reserved, true);
}


static inline uint16_t _ttalink_general_param_request_list_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,uint32_t reserved, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, reserved);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN);
#else
    ttalink_general_param_request_list_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.reserved = reserved;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
}

/**
 * @brief Pack a general_param_request_list message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_request_list_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,uint32_t reserved)
{
    return _ttalink_general_param_request_list_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, reserved, false);
}

/**
 * @brief Pack a general_param_request_list message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_general_param_request_list_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   int64_t sn,uint8_t device_type,uint32_t reserved)
{
    return _ttalink_general_param_request_list_pack_chan(dst_addr, src_addr, chan, msg,  sn, device_type, reserved, true);
}


static inline uint16_t _ttalink_general_param_request_list_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list, bool nocrc)
{
    if(nocrc){
        return ttalink_general_param_request_list_pack_nocrc(dst_addr, src_addr, msg, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }else{
        return ttalink_general_param_request_list_pack(dst_addr, src_addr, msg, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }
    
}

/**
 * @brief Encode a general_param_request_list struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_param_request_list C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_request_list_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list)
{
    return _ttalink_general_param_request_list_encode(dst_addr, src_addr, msg, general_param_request_list, false);
}

/**
 * @brief Encode a general_param_request_list struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param general_param_request_list C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_request_list_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list)
{
    return _ttalink_general_param_request_list_encode(dst_addr, src_addr, msg, general_param_request_list, true);
}


static inline uint16_t _ttalink_general_param_request_list_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list, bool nocrc)
{
    if(nocrc){
        return ttalink_general_param_request_list_pack_chan_nocrc(dst_addr, src_addr, chan, msg, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }else{
        return ttalink_general_param_request_list_pack_chan(dst_addr, src_addr, chan, msg, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }
}

/**
 * @brief Encode a general_param_request_list struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_param_request_list C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_request_list_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list)
{
    return _ttalink_general_param_request_list_encode_chan(dst_addr, src_addr, chan, msg, general_param_request_list, false);
}

/**
 * @brief Encode a general_param_request_list struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param general_param_request_list C-struct to read the message contents from
 */
static inline uint16_t ttalink_general_param_request_list_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_general_param_request_list_t* general_param_request_list)
{
    return _ttalink_general_param_request_list_encode_chan(dst_addr, src_addr, chan, msg, general_param_request_list, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_general_param_request_list_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, uint32_t reserved, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN];
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, reserved);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST, buf, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
#else
    ttalink_general_param_request_list_t packet;
    packet.sn = sn;
    packet.device_type = device_type;
    packet.reserved = reserved;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST, (const char *)&packet, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_param_request_list message
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 */
static inline void ttalink_general_param_request_list_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, uint32_t reserved)
{
    _ttalink_general_param_request_list_send(dst_addr, src_addr, chan, sn, device_type, reserved, false);
}

/**
 * @brief Send a general_param_request_list message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param sn 设备序列号.
 * @param device_type 设备类型.
 * @param reserved This is reserved
 */
static inline void ttalink_general_param_request_list_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, int64_t sn, uint8_t device_type, uint32_t reserved)
{
    _ttalink_general_param_request_list_send(dst_addr, src_addr, chan, sn, device_type, reserved, true);
}


static inline void _ttalink_general_param_request_list_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_request_list_t* general_param_request_list, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_general_param_request_list_send_nocrc(dst_addr, src_addr, chan, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }else{
        ttalink_general_param_request_list_send(dst_addr, src_addr, chan, general_param_request_list->sn, general_param_request_list->device_type, general_param_request_list->reserved);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST, (const char *)general_param_request_list, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
#endif
}

/**
 * @brief Send a general_param_request_list message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_param_request_list_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_request_list_t* general_param_request_list)
{
    _ttalink_general_param_request_list_send_struct(dst_addr, src_addr, chan, general_param_request_list, false);
}

/**
 * @brief Send a general_param_request_list message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_general_param_request_list_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_general_param_request_list_t* general_param_request_list)
{
    _ttalink_general_param_request_list_send_struct(dst_addr, src_addr, chan, general_param_request_list, true);
}

#if TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_general_param_request_list_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, uint32_t reserved, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_int64_t(buf, 0, sn);
    _tta_put_uint8_t(buf, 8, device_type);
    _tta_put_uint32_t(buf, 9, reserved);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST, buf, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
#else
    ttalink_general_param_request_list_t *packet = (ttalink_general_param_request_list_t *)msgbuf;
    packet->sn = sn;
    packet->device_type = device_type;
    packet->reserved = reserved;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST, (const char *)packet, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_MIN_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_general_param_request_list_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, uint32_t reserved)
{
    _ttalink_general_param_request_list_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, reserved, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_general_param_request_list_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  int64_t sn, uint8_t device_type, uint32_t reserved)
{
    _ttalink_general_param_request_list_send_buf(dst_addr, src_addr, msgbuf, chan, sn, device_type, reserved, true);
}
#endif

#endif

// MESSAGE GENERAL_PARAM_REQUEST_LIST UNPACKING


/**
 * @brief Get field sn from general_param_request_list message
 *
 * @return 设备序列号.
 */
static inline int64_t ttalink_general_param_request_list_get_sn(const ttalink_message_t* msg)
{
    return _TTA_RETURN_int64_t(msg,  0);
}

/**
 * @brief Get field device_type from general_param_request_list message
 *
 * @return 设备类型.
 */
static inline uint8_t ttalink_general_param_request_list_get_device_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field reserved from general_param_request_list message
 *
 * @return This is reserved
 */
static inline uint32_t ttalink_general_param_request_list_get_reserved(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Decode a general_param_request_list message into a struct
 *
 * @param msg The message to decode
 * @param general_param_request_list C-struct to decode the message contents into
 */
static inline void ttalink_general_param_request_list_decode(const ttalink_message_t* msg, ttalink_general_param_request_list_t* general_param_request_list)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    general_param_request_list->sn = ttalink_general_param_request_list_get_sn(msg);
    general_param_request_list->device_type = ttalink_general_param_request_list_get_device_type(msg);
    general_param_request_list->reserved = ttalink_general_param_request_list_get_reserved(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN? msg->len : TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN;
        memset(general_param_request_list, 0, TTALINK_MSG_ID_GENERAL_PARAM_REQUEST_LIST_LEN);
    memcpy(general_param_request_list, _TTA_PAYLOAD(msg), len);
#endif
}
