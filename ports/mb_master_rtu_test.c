#include <rtthread.h>
#include "modbus.h"
#include "drv_gpio.h"

//#define MODBUS_USING_RS485

#ifdef MODBUS_USING_RS485
#define RS485_RE GET_PIN(A, 8)
#endif

#define MODBUS_REGISTER_READ_NUM        20

static void modbus_master_thread(void *param)
{
    uint16_t tab_reg[64] = {0};
    modbus_t *ctx = RT_NULL;
    ctx = modbus_new_rtu("uart3", 9600, 'N', 8, 1);
    if (ctx == RT_NULL)
    {
        rt_kprintf("%s : modbus rtu init failed\n", __func__);
        return;
    }
#ifdef MODBUS_USING_RS485
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);
    modbus_rtu_set_rts(ctx, RS485_RE, MODBUS_RTU_RTS_UP);
#else
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS232);
#endif
    modbus_set_slave(ctx, 3); //slave address
    modbus_connect(ctx);
    modbus_set_debug(ctx, 1);
    modbus_set_response_timeout(ctx, 0, 900000);
    int num = 0;
    while (1)
    {
        rt_memset(tab_reg, 0, sizeof(tab_reg));
        int regs = modbus_read_registers(ctx, 0, MODBUS_REGISTER_READ_NUM, tab_reg);
        rt_kprintf("[%04d][read num = %d]\n", num, regs);
        if (regs != -1)
        {
            num++;
            int i;
            for (i = 0; i < MODBUS_REGISTER_READ_NUM; i++)
            {
                rt_kprintf("<%#x>", tab_reg[i]);
            }
            rt_kprintf("\n\n");
        }
        else
        {
            rt_kprintf("read [%d] : %d error\n\n", num, regs);
        }
        rt_thread_mdelay(2000);
    }
    //7-关闭modbus端口
    modbus_close(ctx);

    //8-释放modbus资源
    modbus_free(ctx);
}

static int rtu_test_init(void)
{
#ifdef MODBUS_USING_RS485
    rt_pin_mode(RS485_RE, PIN_MODE_OUTPUT);
#endif
    rt_thread_t tid;
    tid = rt_thread_create("modbus",
                           modbus_master_thread, RT_NULL,
                           2048,
                           12, 10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(rtu_test_init);
