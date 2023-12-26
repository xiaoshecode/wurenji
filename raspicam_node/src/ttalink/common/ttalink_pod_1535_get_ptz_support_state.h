#pragma once
// MESSAGE POD_1535_GET_PTZ_SUPPORT_STATE PACKING

#define TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE 11535

TTAPACKED(
typedef struct __ttalink_pod_1535_get_ptz_support_state_t {
 uint8_t ack; /*< .*/
 uint8_t ack_state; /*< PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.*/
}) ttalink_pod_1535_get_ptz_support_state_t;

#define TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN 2
#define TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN 2
#define TTALINK_MSG_ID_11535_LEN 2
#define TTALINK_MSG_ID_11535_MIN_LEN 2

#define TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC 106
#define TTALINK_MSG_ID_11535_CRC 106



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_POD_1535_GET_PTZ_SUPPORT_STATE { \
    11535, \
    "POD_1535_GET_PTZ_SUPPORT_STATE", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1535_get_ptz_support_state_t, ack) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1535_get_ptz_support_state_t, ack_state) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_POD_1535_GET_PTZ_SUPPORT_STATE { \
    "POD_1535_GET_PTZ_SUPPORT_STATE", \
    2, \
    {  { "ack", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_pod_1535_get_ptz_support_state_t, ack) }, \
         { "ack_state", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_pod_1535_get_ptz_support_state_t, ack_state) }, \
         } \
}
#endif


static inline uint16_t _ttalink_pod_1535_get_ptz_support_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN);
#else
    ttalink_pod_1535_get_ptz_support_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1535_get_ptz_support_state message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_pack(dst_addr, src_addr, msg,  ack, ack_state, false);
}

/**
 * @brief Pack a pod_1535_get_ptz_support_state message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t ack, uint8_t ack_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_pack(dst_addr, src_addr, msg,  ack, ack_state, true);
}


static inline uint16_t _ttalink_pod_1535_get_ptz_support_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN);
#else
    ttalink_pod_1535_get_ptz_support_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
}

/**
 * @brief Pack a pod_1535_get_ptz_support_state message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state, false);
}

/**
 * @brief Pack a pod_1535_get_ptz_support_state message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t ack,uint8_t ack_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_pack_chan(dst_addr, src_addr, chan, msg,  ack, ack_state, true);
}


static inline uint16_t _ttalink_pod_1535_get_ptz_support_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1535_get_ptz_support_state_pack_nocrc(dst_addr, src_addr, msg, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }else{
        return ttalink_pod_1535_get_ptz_support_state_pack(dst_addr, src_addr, msg, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }
    
}

/**
 * @brief Encode a pod_1535_get_ptz_support_state struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1535_get_ptz_support_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_encode(dst_addr, src_addr, msg, pod_1535_get_ptz_support_state, false);
}

/**
 * @brief Encode a pod_1535_get_ptz_support_state struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param pod_1535_get_ptz_support_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_encode(dst_addr, src_addr, msg, pod_1535_get_ptz_support_state, true);
}


static inline uint16_t _ttalink_pod_1535_get_ptz_support_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state, bool nocrc)
{
    if(nocrc){
        return ttalink_pod_1535_get_ptz_support_state_pack_chan_nocrc(dst_addr, src_addr, chan, msg, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }else{
        return ttalink_pod_1535_get_ptz_support_state_pack_chan(dst_addr, src_addr, chan, msg, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }
}

/**
 * @brief Encode a pod_1535_get_ptz_support_state struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1535_get_ptz_support_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_encode_chan(dst_addr, src_addr, chan, msg, pod_1535_get_ptz_support_state, false);
}

/**
 * @brief Encode a pod_1535_get_ptz_support_state struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param pod_1535_get_ptz_support_state C-struct to read the message contents from
 */
static inline uint16_t ttalink_pod_1535_get_ptz_support_state_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    return _ttalink_pod_1535_get_ptz_support_state_encode_chan(dst_addr, src_addr, chan, msg, pod_1535_get_ptz_support_state, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_pod_1535_get_ptz_support_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN];
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE, buf, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
#else
    ttalink_pod_1535_get_ptz_support_state_t packet;
    packet.ack = ack;
    packet.ack_state = ack_state;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE, (const char *)&packet, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1535_get_ptz_support_state message
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send(dst_addr, src_addr, chan, ack, ack_state, false);
}

/**
 * @brief Send a pod_1535_get_ptz_support_state message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param ack .
 * @param ack_state PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t ack, uint8_t ack_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send(dst_addr, src_addr, chan, ack, ack_state, true);
}


static inline void _ttalink_pod_1535_get_ptz_support_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_pod_1535_get_ptz_support_state_send_nocrc(dst_addr, src_addr, chan, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }else{
        ttalink_pod_1535_get_ptz_support_state_send(dst_addr, src_addr, chan, pod_1535_get_ptz_support_state->ack, pod_1535_get_ptz_support_state->ack_state);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE, (const char *)pod_1535_get_ptz_support_state, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
#endif
}

/**
 * @brief Send a pod_1535_get_ptz_support_state message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send_struct(dst_addr, src_addr, chan, pod_1535_get_ptz_support_state, false);
}

/**
 * @brief Send a pod_1535_get_ptz_support_state message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send_struct(dst_addr, src_addr, chan, pod_1535_get_ptz_support_state, true);
}

#if TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_pod_1535_get_ptz_support_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, ack);
    _tta_put_uint8_t(buf, 1, ack_state);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE, buf, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
#else
    ttalink_pod_1535_get_ptz_support_state_t *packet = (ttalink_pod_1535_get_ptz_support_state_t *)msgbuf;
    packet->ack = ack;
    packet->ack_state = ack_state;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE, (const char *)packet, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_MIN_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_pod_1535_get_ptz_support_state_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t ack, uint8_t ack_state)
{
    _ttalink_pod_1535_get_ptz_support_state_send_buf(dst_addr, src_addr, msgbuf, chan, ack, ack_state, true);
}
#endif

#endif

// MESSAGE POD_1535_GET_PTZ_SUPPORT_STATE UNPACKING


/**
 * @brief Get field ack from pod_1535_get_ptz_support_state message
 *
 * @return .
 */
static inline uint8_t ttalink_pod_1535_get_ptz_support_state_get_ack(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ack_state from pod_1535_get_ptz_support_state message
 *
 * @return PTZ support status.
												0 :  Command sets are not supported.
												1 :  Supports command set.
 */
static inline uint8_t ttalink_pod_1535_get_ptz_support_state_get_ack_state(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a pod_1535_get_ptz_support_state message into a struct
 *
 * @param msg The message to decode
 * @param pod_1535_get_ptz_support_state C-struct to decode the message contents into
 */
static inline void ttalink_pod_1535_get_ptz_support_state_decode(const ttalink_message_t* msg, ttalink_pod_1535_get_ptz_support_state_t* pod_1535_get_ptz_support_state)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    pod_1535_get_ptz_support_state->ack = ttalink_pod_1535_get_ptz_support_state_get_ack(msg);
    pod_1535_get_ptz_support_state->ack_state = ttalink_pod_1535_get_ptz_support_state_get_ack_state(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN? msg->len : TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN;
        memset(pod_1535_get_ptz_support_state, 0, TTALINK_MSG_ID_POD_1535_GET_PTZ_SUPPORT_STATE_LEN);
    memcpy(pod_1535_get_ptz_support_state, _TTA_PAYLOAD(msg), len);
#endif
}
