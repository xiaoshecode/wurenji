#pragma once
// MESSAGE CHANNEL_FUNCTION_CONFIG_CMD PACKING

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD 2260

TTAPACKED(
typedef struct __ttalink_channel_function_config_cmd_t {
 uint8_t cmd; /*< 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	*/
 uint8_t chan_type; /*< 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT*/
 uint8_t chan_idx; /*< 通道索引*/
}) ttalink_channel_function_config_cmd_t;

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN 3
#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN 3
#define TTALINK_MSG_ID_2260_LEN 3
#define TTALINK_MSG_ID_2260_MIN_LEN 3

#define TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC 156
#define TTALINK_MSG_ID_2260_CRC 156



#if TTALINK_COMMAND_24BIT
#define TTALINK_MESSAGE_INFO_CHANNEL_FUNCTION_CONFIG_CMD { \
    2260, \
    "CHANNEL_FUNCTION_CONFIG_CMD", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_channel_function_config_cmd_t, cmd) }, \
         { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_channel_function_config_cmd_t, chan_type) }, \
         { "chan_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_channel_function_config_cmd_t, chan_idx) }, \
         } \
}
#else
#define TTALINK_MESSAGE_INFO_CHANNEL_FUNCTION_CONFIG_CMD { \
    "CHANNEL_FUNCTION_CONFIG_CMD", \
    3, \
    {  { "cmd", NULL, TTALINK_TYPE_UINT8_T, 0, 0, offsetof(ttalink_channel_function_config_cmd_t, cmd) }, \
         { "chan_type", NULL, TTALINK_TYPE_UINT8_T, 0, 1, offsetof(ttalink_channel_function_config_cmd_t, chan_type) }, \
         { "chan_idx", NULL, TTALINK_TYPE_UINT8_T, 0, 2, offsetof(ttalink_channel_function_config_cmd_t, chan_idx) }, \
         } \
}
#endif


static inline uint16_t _ttalink_channel_function_config_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t chan_type, uint8_t chan_idx, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, chan_type);
    _tta_put_uint8_t(buf, 2, chan_idx);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN);
#else
    ttalink_channel_function_config_cmd_t packet;
    packet.cmd = cmd;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD;
    return ttalink_finalize_message(msg, dst_addr, src_addr, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
}

/**
 * @brief Pack a channel_function_config_cmd message
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_channel_function_config_cmd_pack(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    return _ttalink_channel_function_config_cmd_pack(dst_addr, src_addr, msg,  cmd, chan_type, chan_idx, false);
}

/**
 * @brief Pack a channel_function_config_cmd message, no crc
 * @param dst_addr 
 * @param src_addr
 * @param msg The TTAlink message to compress the data into
 *
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_channel_function_config_cmd_pack_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg,
                               uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    return _ttalink_channel_function_config_cmd_pack(dst_addr, src_addr, msg,  cmd, chan_type, chan_idx, true);
}


static inline uint16_t _ttalink_channel_function_config_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t chan_type,uint8_t chan_idx, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, chan_type);
    _tta_put_uint8_t(buf, 2, chan_idx);

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN);
#else
    ttalink_channel_function_config_cmd_t packet;
    packet.cmd = cmd;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;

        memcpy(_TTA_PAYLOAD_NON_CONST(msg), &packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN);
#endif

    msg->msgid = TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD;
    return ttalink_finalize_message_chan(msg, dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
}

/**
 * @brief Pack a channel_function_config_cmd message on a channel
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_channel_function_config_cmd_pack_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t chan_type,uint8_t chan_idx)
{
    return _ttalink_channel_function_config_cmd_pack_chan(dst_addr, src_addr, chan, msg,  cmd, chan_type, chan_idx, false);
}

/**
 * @brief Pack a channel_function_config_cmd message on a channel, no crc
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t ttalink_channel_function_config_cmd_pack_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan,
                               ttalink_message_t* msg,
                                   uint8_t cmd,uint8_t chan_type,uint8_t chan_idx)
{
    return _ttalink_channel_function_config_cmd_pack_chan(dst_addr, src_addr, chan, msg,  cmd, chan_type, chan_idx, true);
}


static inline uint16_t _ttalink_channel_function_config_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_channel_function_config_cmd_pack_nocrc(dst_addr, src_addr, msg, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }else{
        return ttalink_channel_function_config_cmd_pack(dst_addr, src_addr, msg, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }
    
}

/**
 * @brief Encode a channel_function_config_cmd struct
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_cmd_encode(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    return _ttalink_channel_function_config_cmd_encode(dst_addr, src_addr, msg, channel_function_config_cmd, false);
}

/**
 * @brief Encode a channel_function_config_cmd struct, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_cmd_encode_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    return _ttalink_channel_function_config_cmd_encode(dst_addr, src_addr, msg, channel_function_config_cmd, true);
}


static inline uint16_t _ttalink_channel_function_config_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd, bool nocrc)
{
    if(nocrc){
        return ttalink_channel_function_config_cmd_pack_chan_nocrc(dst_addr, src_addr, chan, msg, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }else{
        return ttalink_channel_function_config_cmd_pack_chan(dst_addr, src_addr, chan, msg, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }
}

/**
 * @brief Encode a channel_function_config_cmd struct on a channel
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_cmd_encode_chan(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    return _ttalink_channel_function_config_cmd_encode_chan(dst_addr, src_addr, chan, msg, channel_function_config_cmd, false);
}

/**
 * @brief Encode a channel_function_config_cmd struct on a channel, no crc
 *
 * @param dst_addr ID of this system
 * @param src_addr ID of this component (e.g. 200 for IMU)
 * @param chan The TTAlink channel this message will be sent over
 * @param msg The TTAlink message to compress the data into
 * @param channel_function_config_cmd C-struct to read the message contents from
 */
static inline uint16_t ttalink_channel_function_config_cmd_encode_chan_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, ttalink_message_t* msg, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    return _ttalink_channel_function_config_cmd_encode_chan(dst_addr, src_addr, chan, msg, channel_function_config_cmd, true);
}


#ifdef TTALINK_USE_CONVENIENCE_FUNCTIONS

static inline void _ttalink_channel_function_config_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t chan_type, uint8_t chan_idx, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char buf[TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN];
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, chan_type);
    _tta_put_uint8_t(buf, 2, chan_idx);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD, buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
#else
    ttalink_channel_function_config_cmd_t packet;
    packet.cmd = cmd;
    packet.chan_type = chan_type;
    packet.chan_idx = chan_idx;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD, (const char *)&packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a channel_function_config_cmd message
 * @param chan TTAlink channel to send the message
 *
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 */
static inline void ttalink_channel_function_config_cmd_send(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    _ttalink_channel_function_config_cmd_send(dst_addr, src_addr, chan, cmd, chan_type, chan_idx, false);
}

/**
 * @brief Send a channel_function_config_cmd message, no crc
 * @param chan TTAlink channel to send the message
 *
 * @param cmd 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 * @param chan_type 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 * @param chan_idx 通道索引
 */
static inline void ttalink_channel_function_config_cmd_send_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    _ttalink_channel_function_config_cmd_send(dst_addr, src_addr, chan, cmd, chan_type, chan_idx, true);
}


static inline void _ttalink_channel_function_config_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    if(nocrc){
        ttalink_channel_function_config_cmd_send_nocrc(dst_addr, src_addr, chan, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }else{
        ttalink_channel_function_config_cmd_send(dst_addr, src_addr, chan, channel_function_config_cmd->cmd, channel_function_config_cmd->chan_type, channel_function_config_cmd->chan_idx);
    }
#else
    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD, (const char *)channel_function_config_cmd, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
#endif
}

/**
 * @brief Send a channel_function_config_cmd message
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_channel_function_config_cmd_send_struct(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    _ttalink_channel_function_config_cmd_send_struct(dst_addr, src_addr, chan, channel_function_config_cmd, false);
}

/**
 * @brief Send a channel_function_config_cmd message, no crc
 * @param chan TTAlink channel to send the message
 * @param struct The TTAlink struct to serialize
 */
static inline void ttalink_channel_function_config_cmd_send_struct_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_channel_t chan, const ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
    _ttalink_channel_function_config_cmd_send_struct(dst_addr, src_addr, chan, channel_function_config_cmd, true);
}

#if TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN <= TTALINK_MAX_PAYLOAD_LEN
static inline void _ttalink_channel_function_config_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t chan_type, uint8_t chan_idx, bool nocrc)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _tta_put_uint8_t(buf, 0, cmd);
    _tta_put_uint8_t(buf, 1, chan_type);
    _tta_put_uint8_t(buf, 2, chan_idx);

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD, buf, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
#else
    ttalink_channel_function_config_cmd_t *packet = (ttalink_channel_function_config_cmd_t *)msgbuf;
    packet->cmd = cmd;
    packet->chan_type = chan_type;
    packet->chan_idx = chan_idx;

    _tta_finalize_message_chan_send(dst_addr, src_addr, chan, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD, (const char *)packet, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_MIN_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_CRC, nocrc);
#endif
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void ttalink_channel_function_config_cmd_send_buf(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    _ttalink_channel_function_config_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, chan_type, chan_idx, false);
}

/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  ttalink_message_t which is the size of a full ttalink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage, no crc.
 */
static inline void ttalink_channel_function_config_cmd_send_buf_nocrc(uint8_t dst_addr, uint8_t src_addr, ttalink_message_t *msgbuf, ttalink_channel_t chan,  uint8_t cmd, uint8_t chan_type, uint8_t chan_idx)
{
    _ttalink_channel_function_config_cmd_send_buf(dst_addr, src_addr, msgbuf, chan, cmd, chan_type, chan_idx, true);
}
#endif

#endif

// MESSAGE CHANNEL_FUNCTION_CONFIG_CMD UNPACKING


/**
 * @brief Get field cmd from channel_function_config_cmd message
 *
 * @return 
        		1： 读取全部（返回：#2258：CHANNEL_FUNCTION_CONFIG）
                2： 读取单个（返回：#2259：SINGLE_CHANNEL_FUNCTION_CONFIG）
                3： 设置全部
                4： 设置单个
        	
 */
static inline uint8_t ttalink_channel_function_config_cmd_get_cmd(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field chan_type from channel_function_config_cmd message
 *
 * @return 通道类型： 0：无； 1：RC_INPUT； 2：PWM_OUTPUT
 */
static inline uint8_t ttalink_channel_function_config_cmd_get_chan_type(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field chan_idx from channel_function_config_cmd message
 *
 * @return 通道索引
 */
static inline uint8_t ttalink_channel_function_config_cmd_get_chan_idx(const ttalink_message_t* msg)
{
    return _TTA_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a channel_function_config_cmd message into a struct
 *
 * @param msg The message to decode
 * @param channel_function_config_cmd C-struct to decode the message contents into
 */
static inline void ttalink_channel_function_config_cmd_decode(const ttalink_message_t* msg, ttalink_channel_function_config_cmd_t* channel_function_config_cmd)
{
#if TTALINK_NEED_BYTE_SWAP || !TTALINK_ALIGNED_FIELDS
    channel_function_config_cmd->cmd = ttalink_channel_function_config_cmd_get_cmd(msg);
    channel_function_config_cmd->chan_type = ttalink_channel_function_config_cmd_get_chan_type(msg);
    channel_function_config_cmd->chan_idx = ttalink_channel_function_config_cmd_get_chan_idx(msg);
#else
        uint8_t len = msg->len < TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN? msg->len : TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN;
        memset(channel_function_config_cmd, 0, TTALINK_MSG_ID_CHANNEL_FUNCTION_CONFIG_CMD_LEN);
    memcpy(channel_function_config_cmd, _TTA_PAYLOAD(msg), len);
#endif
}
