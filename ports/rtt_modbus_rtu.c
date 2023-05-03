/*
 * Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1+
 */

#include "modbus-private.h"
#include "modbus-rtu.h"
#include "modbus-rtu-private.h"

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* Define the slave ID of the remote device to talk in master mode or set the
 * internal slave ID in slave mode */
static int _modbus_set_slave(modbus_t *ctx, int slave)
{
    /* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
    if (slave >= 0 && slave <= 247) {
        ctx->slave = slave;
    } else {
        errno = EINVAL;
        return -1;
    }

    return 0;
}

/* Builds a RTU request header */
static int _modbus_rtu_build_request_basis(modbus_t *ctx, int function,
                                           int addr, int nb,
                                           uint8_t *req)
{
    RT_ASSERT(ctx->slave != -1);
    req[0] = ctx->slave;
    req[1] = function;
    req[2] = addr >> 8;
    req[3] = addr & 0x00ff;
    req[4] = nb >> 8;
    req[5] = nb & 0x00ff;

    return _MODBUS_RTU_PRESET_REQ_LENGTH;
}

/* Builds a RTU response header */
static int _modbus_rtu_build_response_basis(sft_t *sft, uint8_t *rsp)
{
    /* In this case, the slave is certainly valid because a check is already
     * done in _modbus_rtu_listen */
    rsp[0] = sft->slave;
    rsp[1] = sft->function;

    return _MODBUS_RTU_PRESET_RSP_LENGTH;
}

static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

static int _modbus_rtu_prepare_response_tid(const uint8_t *req, int *req_length)
{
    (*req_length) -= _MODBUS_RTU_CHECKSUM_LENGTH;
    /* No TID */
    return 0;
}

static int _modbus_rtu_send_msg_pre(uint8_t *req, int req_length)
{
    uint16_t crc = crc16(req, req_length);
    req[req_length++] = crc >> 8;
    req[req_length++] = crc & 0x00FF;

    return req_length;
}

#if HAVE_DECL_TIOCM_RTS
static void _modbus_rtu_ioctl_rts(modbus_t *ctx, int on)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    if (on) {
        rt_pin_write(ctx_rtu->rts_pin, PIN_HIGH);
    } else {
        rt_pin_write(ctx_rtu->rts_pin, PIN_LOW);
    }
}
#endif

static ssize_t _modbus_rtu_send(modbus_t *ctx, const uint8_t *req, int req_length)
{
    struct rt_device *dev;

    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    dev = (struct rt_device *)ctx_rtu->data;
    if (dev == RT_NULL)
    {
        rt_kprintf("modbus rtu device not init\n");
        return 0;
    }

#if HAVE_DECL_TIOCM_RTS
    if (ctx_rtu->rts != MODBUS_RTU_RTS_NONE)
    {
        ssize_t size;

        if (ctx->debug)
        {
            rt_kprintf("Sending request using RTS signal\n");
        }

        ctx_rtu->set_rts(ctx, ctx_rtu->rts == MODBUS_RTU_RTS_UP);
        rt_int32_t nms = (ctx_rtu->rts_delay/1000) ? (ctx_rtu->rts_delay/1000) : 1;
        rt_thread_mdelay(nms);

        size = rt_device_write(dev, 0, req, req_length);

        ctx_rtu->set_rts(ctx, ctx_rtu->rts != MODBUS_RTU_RTS_UP);

        return size;
    }
    else
    {
#endif
        return rt_device_write(dev, 0, req, req_length);
#if HAVE_DECL_TIOCM_RTS
    }
#endif
}

static int _modbus_rtu_receive(modbus_t *ctx, uint8_t *req)
{
    return _modbus_receive_msg(ctx, req, MSG_INDICATION);
}

static ssize_t _modbus_rtu_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length)
{
    struct rt_device *dev;

    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    dev = (struct rt_device *)ctx_rtu->data;
    if (dev == RT_NULL)
    {
        rt_kprintf("modbus rtu device not init\n");
        return 0;
    }

    return rt_device_read(dev, 0, rsp, rsp_length);
}

static int _modbus_rtu_flush(modbus_t *);

static int _modbus_rtu_pre_check_confirmation(modbus_t *ctx, const uint8_t *req,
                                              const uint8_t *rsp, int rsp_length)
{
    /* Check responding slave is the slave we requested (except for broacast
     * request) */
    if (req[0] != rsp[0] && req[0] != MODBUS_BROADCAST_ADDRESS)
    {
        if (ctx->debug)
        {
            rt_kprintf("The responding slave %d isn't the requested slave %d\n",
                    rsp[0], req[0]);
        }
        errno = EMBBADSLAVE;
        return -1;
    } else {
        return 0;
    }
}

/* The check_crc16 function shall return 0 is the message is ignored and the
   message length if the CRC is valid. Otherwise it shall return -1 and set
   errno to EMBADCRC. */
static int _modbus_rtu_check_integrity(modbus_t *ctx, uint8_t *msg,
                                       const int msg_length)
{
    uint16_t crc_calculated;
    uint16_t crc_received;

    crc_calculated = crc16(msg, msg_length - 2);
    crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

    /* Check CRC of msg */
    if (crc_calculated == crc_received)
    {
        return msg_length;
    }
    else
    {
        if (ctx->debug)
        {
            rt_kprintf("ERROR CRC received 0x%0X != CRC calculated 0x%0X\n",
                    crc_received, crc_calculated);
        }

        if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
            _modbus_rtu_flush(ctx);
        }
        errno = EMBBADCRC;
        return -1;
    }
}

/* Sets up a serial port for RTU communications */
static int _modbus_rtu_connect(modbus_t *ctx)
{
    rt_err_t ret;
    struct rt_device *dev;
    struct serial_configure rtu_serial_config = RT_SERIAL_CONFIG_DEFAULT;

    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    dev = (struct rt_device *)ctx_rtu->data;
    if (dev == RT_NULL)
    {
        rt_kprintf("modbus rtu device not init\n");
        errno = EINVAL;
        return -1;
    }

    ret = rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (ret != RT_EOK)
    {
        rt_kprintf("The modbus rtu device open failed\n");
        errno = EINVAL;
        return -1;
    }

    rtu_serial_config.baud_rate = ctx_rtu->baud;
    rtu_serial_config.parity = ctx_rtu->parity;
    rtu_serial_config.stop_bits = ctx_rtu->stop_bit;
    rtu_serial_config.data_bits = ctx_rtu->data_bit;

    ret = rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &rtu_serial_config);
    if (ret != RT_EOK)
    {
        rt_kprintf("The modbus rtu device config failed\n");
        errno = EINVAL;
        return -1;
    }

    ctx_rtu->connected = 1;
    return 0;
}

unsigned int _modbus_rtu_is_connected(modbus_t *ctx)
{
    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    if (ctx_rtu == RT_NULL)
    {
        rt_kprintf("modbus rtu device not init\n");
        errno = EINVAL;
        return 0;
    }

    return ctx_rtu->connected;
}

int modbus_rtu_set_serial_mode(modbus_t *ctx, int mode)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCSRS485
        modbus_rtu_t *ctx_rtu = ctx->backend_data;

        if (mode == MODBUS_RTU_RS485) {
            ctx_rtu->serial_mode = MODBUS_RTU_RS485;
            return 0;
        } else if (mode == MODBUS_RTU_RS232) {
            /* Turn off RS485 mode only if required */
            if (ctx_rtu->serial_mode == MODBUS_RTU_RS485) {
            }
            ctx_rtu->serial_mode = MODBUS_RTU_RS232;
            return 0;
        }
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    }

    /* Wrong backend and invalid mode specified */
    errno = EINVAL;
    return -1;
}

int modbus_rtu_get_serial_mode(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCSRS485
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
        return ctx_rtu->serial_mode;
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    } else {
        errno = EINVAL;
        return -1;
    }
}

int modbus_rtu_get_rts(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCM_RTS
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
        return ctx_rtu->rts;
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    } else {
        errno = EINVAL;
        return -1;
    }
}

int modbus_rtu_set_rts(modbus_t *ctx, int rts_pin, int mode)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCM_RTS
        modbus_rtu_t *ctx_rtu = ctx->backend_data;

        if (mode == MODBUS_RTU_RTS_NONE || mode == MODBUS_RTU_RTS_UP ||
            mode == MODBUS_RTU_RTS_DOWN) {
            ctx_rtu->rts_pin = rts_pin;
                
            ctx_rtu->rts = mode;

            /* Set the RTS bit in order to not reserve the RS485 bus */
            ctx_rtu->set_rts(ctx, ctx_rtu->rts != MODBUS_RTU_RTS_UP);

            return 0;
        } else {
            errno = EINVAL;
            return -1;
        }
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    }
    /* Wrong backend or invalid mode specified */
    errno = EINVAL;
    return -1;
}

int modbus_rtu_set_custom_rts(modbus_t *ctx, void (*set_rts) (modbus_t *ctx, int on))
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCM_RTS
        modbus_rtu_t *ctx_rtu = ctx->backend_data;
        ctx_rtu->set_rts = set_rts;
        return 0;
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    } else {
        errno = EINVAL;
        return -1;
    }
}

int modbus_rtu_get_rts_delay(modbus_t *ctx)
{
    if (ctx == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCM_RTS
        modbus_rtu_t *ctx_rtu;
        ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        return ctx_rtu->rts_delay;
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    } else {
        errno = EINVAL;
        return -1;
    }
}

int modbus_rtu_set_rts_delay(modbus_t *ctx, int us)
{
    if (ctx == NULL || us < 0) {
        errno = EINVAL;
        return -1;
    }

    if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCM_RTS
        modbus_rtu_t *ctx_rtu;
        ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
        ctx_rtu->rts_delay = us;
        return 0;
#else
        if (ctx->debug)
        {
            rt_kprintf("This function isn't supported on your platform\n");
        }
        errno = ENOTSUP;
        return -1;
#endif
    } else {
        errno = EINVAL;
        return -1;
    }
}

static void _modbus_rtu_close(modbus_t *ctx)
{
    struct rt_device *dev;

    modbus_rtu_t *ctx_rtu = ctx->backend_data;
    dev = (struct rt_device *)ctx_rtu->data;
    if (dev == RT_NULL)
    {
        rt_kprintf("modbus rtu device not init\n");
    }
    else
    {
        rt_device_close(dev);
    }
    ctx_rtu->connected = 0;
}

static int _modbus_rtu_flush(modbus_t *ctx)
{
    return 0;
}

static int _modbus_rtu_select(modbus_t *ctx, fd_set *rset,
                              struct timeval *tv, int length_to_read)
{
    return 0;
}

static void _modbus_rtu_free(modbus_t *ctx)
{
    rt_free(((modbus_rtu_t*)ctx->backend_data)->device);
    rt_free(ctx->backend_data);
    rt_free(ctx);
}

const modbus_backend_t _modbus_rtu_backend = {
    .backend_type = _MODBUS_BACKEND_TYPE_RTU,
    .header_length = _MODBUS_RTU_HEADER_LENGTH,
    .checksum_length = _MODBUS_RTU_CHECKSUM_LENGTH,
    .max_adu_length = MODBUS_RTU_MAX_ADU_LENGTH,
    .set_slave = _modbus_set_slave,
    .build_request_basis = _modbus_rtu_build_request_basis,
    .build_response_basis = _modbus_rtu_build_response_basis,
    .prepare_response_tid = _modbus_rtu_prepare_response_tid,
    .send_msg_pre = _modbus_rtu_send_msg_pre,
    .send = _modbus_rtu_send,
    .receive = _modbus_rtu_receive,
    .recv = _modbus_rtu_recv,
    .check_integrity = _modbus_rtu_check_integrity,
    .pre_check_confirmation = _modbus_rtu_pre_check_confirmation,
    .connect = _modbus_rtu_connect,
    .is_connected = _modbus_rtu_is_connected,
    .close = _modbus_rtu_close,
    .flush = _modbus_rtu_flush,
    .select = _modbus_rtu_select,
    .free = _modbus_rtu_free,
};

modbus_t* modbus_new_rtu(const char *device_name,
                         int baud, char parity, int data_bit,
                         int stop_bit)
{
    modbus_t *ctx;
    modbus_rtu_t *ctx_rtu;
    struct rt_device *dev = RT_NULL;

    dev = rt_device_find(device_name);
    if (dev == RT_NULL)
    {
        rt_kprintf("The modbus rtu device not found\n");
        errno = EINVAL;
        return NULL;
    }

    /* Check device argument */
    if (device_name == NULL || *device_name == 0)
    {
        rt_kprintf("The device string is empty\n");
        errno = EINVAL;
        return NULL;
    }

    /* Check baud argument */
    if (baud == 0)
    {
        rt_kprintf("The baud rate value must not be zero\n");
        errno = EINVAL;
        return NULL;
    }

    ctx = (modbus_t *)rt_malloc(sizeof(modbus_t));
    _modbus_init_common(ctx);
    ctx->backend = &_modbus_rtu_backend;
    ctx->backend_data = (modbus_rtu_t *)rt_malloc(sizeof(modbus_rtu_t));
    ctx_rtu = (modbus_rtu_t *)ctx->backend_data;
    ctx_rtu->device = NULL;

    /* Device name and \0 */
    ctx_rtu->device = (char *)rt_malloc((rt_strlen(device_name) + 1) * sizeof(char));
    rt_strcpy(ctx_rtu->device, device_name);

    ctx_rtu->data = dev; /* rt_device */

    ctx_rtu->baud = baud;
    if (parity == 'N')
    {
        ctx_rtu->parity = PARITY_NONE;
    }
    else if (parity == 'E')
    {
        ctx_rtu->parity = PARITY_EVEN;
    }
    else if (parity == 'O')
    {
        ctx_rtu->parity = PARITY_ODD;
    }
    else
    {
        modbus_free(ctx);
        errno = EINVAL;
        return NULL;
    }
    ctx_rtu->data_bit = data_bit;
    ctx_rtu->stop_bit = stop_bit;

#if HAVE_DECL_TIOCSRS485
    /* The RS232 mode has been set by default */
    ctx_rtu->serial_mode = MODBUS_RTU_RS232;
#endif

#if HAVE_DECL_TIOCM_RTS
    /* The RTS use has been set by default */
    ctx_rtu->rts = MODBUS_RTU_RTS_NONE;

    /* Calculate estimated time in micro second to send one byte */
    ctx_rtu->onebyte_time = 1000000 * (1 + data_bit + (parity == 'N' ? 0 : 1) + stop_bit) / baud;
    
    ctx_rtu->rts_pin = 0;
    
    /* The internal function is used by default to set RTS */
    ctx_rtu->set_rts = _modbus_rtu_ioctl_rts;

    /* The delay before and after transmission when toggling the RTS pin */
    ctx_rtu->rts_delay = ctx_rtu->onebyte_time;
#endif

    ctx_rtu->connected = 0;

    return ctx;
}
