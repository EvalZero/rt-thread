/*
 * File      : nuc97x_lcd.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author		Notes
 * 2017/11/09     EvalZero	first version
 */
#include <rthw.h>
#include <rtthread.h>
#include <drivers/spi.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "nuc97x.h"
#include "nuc97x_conf.h"
#include "nuc97x_sys.h"
#include "nuc97x_spi.h"


#define spi_out(dev, byte, addr)        outpw((dev)->base + addr, byte)
#define spi_in(dev, addr)               inpw((dev)->base + addr)

typedef struct {
    uint32_t base;      /* spi bus number */
    uint8_t openflag;
    uint8_t intflag;
} spi_dev;

static spi_dev spi_device[SPI_NUMBER];

/**
  * @brief  SPI-0 Interrupt handler
  * @param None
  * @return None
  */
static void spi0ISR(void)
{
    // clear interrupt flag
    outpw(SPI_REG_CNTRL, 0x1 << 16);
    spi_device[0].intflag = 1;
}

/**
  * @brief  SPI-1 Interrupt handler
  * @param None
  * @return None
  */
static void spi1ISR(void)
{
    // clear interrupt flag
    outpw(SPI_REG_CNTRL, 0x1 << 16);
    spi_device[1].intflag = 1;
}

/**
  * @brief  Set SPI divider
  * @param[in] dev pointer to spi interface structure
  * @param[in] speed desire spi speed
  * @return speed set actually
  */
static uint32_t spiSetSpeed(spi_dev * dev, uint32_t speed)
{
    uint16_t div = (uint16_t)(SPI_INPUT_CLOCK / (2 * speed)) - 1;

    spi_out(dev, div, SPI_REG_DIVIDER);
    return ( SPI_INPUT_CLOCK /  (2*(div+1)));
}

/// @endcond /* HIDDEN_SYMBOLS */

/**
  * @brief Initialize spi interface and install interrupt callback function
  * @return always 0.
  * @retval 0 Success.
  */
int32_t  spiInit(int32_t fd)
{
    if(fd == 0) {
        rt_hw_interrupt_install(SPI0_IRQn, (rt_isr_handler_t)spi0ISR, NULL, "spi0_irq");
        rt_hw_interrupt_umask(SPI0_IRQn);
        // sysInstallISR(IRQ_LEVEL_1, SPI0_IRQn, (PVOID)spi0ISR);
        // sysEnableInterrupt(SPI0_IRQn);
        memset((void *)&spi_device[0], 0, sizeof(spi_dev));
    } else {
        rt_hw_interrupt_install(SPI1_IRQn, (rt_isr_handler_t)spi1ISR, NULL, "spi1_irq");
        rt_hw_interrupt_umask(SPI1_IRQn);
        // sysInstallISR(IRQ_LEVEL_1, SPI1_IRQn, (PVOID)spi1ISR);
        // sysEnableInterrupt(SPI1_IRQn);
        memset((void *)&spi_device[1], 0, sizeof(spi_dev));
    }

    rt_hw_local_interrupt_set(ENABLE_IRQ);
    // sysSetLocalInterrupt(ENABLE_IRQ);

    return(0);
}

/**
  * @brief Support some spi driver commands for application.
  * @param[in] fd is interface number.
  * @param[in] cmd is command.
  * @param[in] arg0 is the first argument of command.
  * @param[in] arg1 is the second argument of command.
  * @return command status.
  * @retval 0 Success otherwise fail. Fail value could be
  *                                    - \ref SPI_ERR_NODEV
  *                                    - \ref SPI_ERR_IO
  *                                    - \ref SPI_ERR_ARG
  */
int32_t spiIoctl(int32_t fd, uint32_t cmd, uint32_t arg0, uint32_t arg1)
{
    spi_dev *dev;

    if(fd != 0 && fd != 1)
        return(SPI_ERR_NODEV);

    dev = (spi_dev *)((uint32_t)&spi_device[fd]);
    if(dev->openflag == 0)
        return(SPI_ERR_IO);

    switch(cmd) {
    case SPI_IOC_TRIGGER:
        spi_out(dev, spi_in(dev, SPI_REG_CNTRL) | 0x1 ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_SET_INTERRUPT:
        if(arg0 == SPI_ENABLE_INTERRUPT)
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) | (0x1<<17) ,SPI_REG_CNTRL);
        else
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) & ~(0x1<<17) ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_SET_SPEED:
        spiSetSpeed(dev, (uint32_t)arg0);
        break;

    case SPI_IOC_SET_DUAL_QUAD_MODE:
        if(arg0 == SPI_DISABLE_DUAL_QUAD) {
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~(0x3 << 21)) ,SPI_REG_CNTRL);
            break;
        }

        if(arg0 == SPI_DUAL_MODE)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~(0x3 << 21)) | (0x1 << 22) ,SPI_REG_CNTRL);
        else
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~(0x3 << 21)) | (0x1 << 21) ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_SET_DUAL_QUAD_DIR:
        if(arg0 == SPI_DUAL_QUAD_INPUT)
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) & ~(0x1 << 20) ,SPI_REG_CNTRL);
        else
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) | (0x1 << 20) ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_SET_LSB_MSB:
        if(arg0 == SPI_MSB)
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) & ~(0x1 << 10) ,SPI_REG_CNTRL);
        else
            spi_out(dev, spi_in(dev, SPI_REG_CNTRL) | (0x1 << 10) ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_SET_TX_NUM:
        if(arg0 < 4)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~(0x3 << 8)) | (arg0 << 8) ,SPI_REG_CNTRL);
        else
            return SPI_ERR_ARG;
        break;

    case SPI_IOC_SET_TX_BITLEN:
        if(arg0 < 32)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~(0x1f << 3)) | (arg0 << 3) ,SPI_REG_CNTRL);
        else
            return SPI_ERR_ARG;
        break;

    case SPI_IOC_SET_MODE:
        if(arg0 > SPI_MODE_3)
            return SPI_ERR_ARG;

        if(arg0 == SPI_MODE_0)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~((0x3<<1) | (1UL<<31))) | (1<<2) ,SPI_REG_CNTRL);
        else if(arg0 == SPI_MODE_1)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~((0x3<<1) | (1UL<<31))) | (1<<1) ,SPI_REG_CNTRL);
        else if(arg0 == SPI_MODE_2)
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~((0x3<<1) | (1UL<<31))) | ((1UL<<31) | (1<<2)) ,SPI_REG_CNTRL);
        else
            spi_out(dev, (spi_in(dev, SPI_REG_CNTRL) & ~((0x3<<1) | (1UL<<31))) | ((1UL<<31) | (1<<1)) ,SPI_REG_CNTRL);
        break;

    case SPI_IOC_ENABLE_SS:
        if(arg0 == SPI_SS_SS0)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x3)) | 0x1 ,SPI_REG_SSR);
        else if(arg0 == SPI_SS_SS1)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x3)) | 0x2 ,SPI_REG_SSR);
        else if(arg0 == SPI_SS_BOTH)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x3)) | 0x3 ,SPI_REG_SSR);
        else
            return SPI_ERR_ARG;
        break;

    case SPI_IOC_DISABLE_SS:
        if(arg0 == SPI_SS_SS0)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x1)) ,SPI_REG_SSR);
        else if(arg0 == SPI_SS_SS1)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x2)) ,SPI_REG_SSR);
        else if(arg0 == SPI_SS_BOTH)
            spi_out(dev, (spi_in(dev, SPI_REG_SSR) & ~(0x3)) ,SPI_REG_SSR);
        else
            return SPI_ERR_ARG;
        break;

    case SPI_IOC_SET_AUTOSS:
        if(arg0 == SPI_DISABLE_AUTOSS)
            spi_out(dev, spi_in(dev, SPI_REG_SSR) & ~(0x1 << 3) ,SPI_REG_SSR);
        else
            spi_out(dev, spi_in(dev, SPI_REG_SSR) | (0x1 << 3) ,SPI_REG_SSR);
        break;

    case SPI_IOC_SET_SS_ACTIVE_LEVEL:
        if(arg0 == SPI_SS_ACTIVE_LOW)
            spi_out(dev, spi_in(dev, SPI_REG_SSR) & ~(0x1 << 2) ,SPI_REG_SSR);
        else
            spi_out(dev, spi_in(dev, SPI_REG_SSR) | (0x1 << 2) ,SPI_REG_SSR);
    default:
        break;
    }

    return 0;
}

/**
  * @brief Open spi interface and initialize some variables
  * @param[in] fd is interface number.
  * @return always 0
  * @retval 0 success.
  */
int spiOpen(int32_t fd)
{
    spi_dev *dev;

    if( (uint32_t)fd >= SPI_NUMBER)
        return SPI_ERR_NODEV;

    dev = (spi_dev *)((uint32_t)&spi_device[fd]);

    if( dev->openflag != 0 )        /* a card slot can open only once */
        return(SPI_ERR_BUSY);

    /* Enable engine clock */
    if((uint32_t)fd == 0)
        outpw(REG_CLK_PCLKEN1, inpw(REG_CLK_PCLKEN1) | 0x10);
    else
        outpw(REG_CLK_PCLKEN1, inpw(REG_CLK_PCLKEN1) | 0x20);

    memset(dev, 0, sizeof(spi_dev));
    dev->base = ((uint32_t)fd) ? SPI1_BA : SPI0_BA;
    dev->openflag = 1;
    dev->intflag = 0;

    return 0;
}

/**
  * @brief Get busy status of spi interface
  * @param[in] fd is interface number.
  * @return busy or not
  * @retval 0 not busy.
  * @retval 1 busy.
  */
uint8_t spiGetBusyStatus(int32_t fd)
{
    spi_dev *dev;

    dev = (spi_dev *)((uint32_t)&spi_device[fd]);

    if(spi_in(dev, SPI_REG_CNTRL) & (0x1 << 17))
        return dev->intflag;
    else
        return (( spi_in(dev, SPI_REG_CNTRL) & 0x1) == 0x1 ? 1:0);
}

/**
  * @brief Read data form spi interface
  * @param[in] fd is interface number.
  * @param[in] buff_id is buffer number. If transfer number is 4, application needs read 4 times (buff_id is from 0 to 3) from buffer.
  * @return data
  */
uint32_t spiRead(int32_t fd, uint8_t buff_id)
{
    spi_dev *dev;

    dev = (spi_dev *)((uint32_t)&spi_device[fd]);
    return spi_in(dev, (SPI_REG_RX0+4*buff_id));
}

/**
  * @brief Write data to spi interface
  * @param[in] fd is interface number.
  * @param[in] buff_id is buffer number. If transfer number is 4, application needs write 4 times (buff_id is from 0 to 3) to buffer.
  * @param[in] data is data to be written.
  * @return none
  */
void spiWrite(int32_t fd, uint8_t buff_id, uint32_t data)
{
    spi_dev *dev;

    dev = (spi_dev *)((uint32_t)&spi_device[fd]);
    spi_out(dev, data, (SPI_REG_TX0+4*buff_id));
}
#if 1

rt_uint8_t nuc97x_transmit_byte(rt_int32_t fd, rt_uint8_t byte)
{
    rt_uint8_t val;

    spiWrite(fd, 0, byte);
    spiIoctl(0, SPI_IOC_TRIGGER, 0, 0);
    while(spiGetBusyStatus(0));
    val = spiRead(fd, 0);

    return val;
}
/**
 * RT-Thread SPI Interface
 */

rt_err_t nuc97x_spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *cfg)
{
    rt_uint8_t mode       = cfg->mode & 0x3;
    rt_uint8_t data_width = cfg->data_width;
    rt_uint32_t max_hz    = cfg->max_hz;
#if 0
    /* 配置模式 */
    spiIoctl(0, SPI_IOC_SET_MODE, mode, 0);

    /* 配置数据宽度 */
    spiIoctl(0, SPI_IOC_SET_TX_BITLEN, data_width, 0);

    /* 配置传输速率 */
    spiIoctl(0, SPI_IOC_SET_SPEED, max_hz, 0);

    rt_kprintf("mode=%d \n", cfg->mode);
    rt_kprintf("max_hz=%d \n", cfg->max_hz);
    rt_kprintf("data width=%d \n", cfg->data_width);
#endif
    return RT_EOK;
}

rt_uint32_t nuc97x_spi_xfer(struct rt_spi_device *device, struct rt_spi_message *msg)
{
    rt_uint32_t len = msg->length;
    rt_uint8_t *send_buf = (rt_uint8_t *)msg->send_buf;
    rt_uint8_t *recv_buf = (rt_uint8_t *)msg->recv_buf;

    struct rt_spi_configuration * cfg = &device->config;

   //  rt_kprintf("msg length=%d \n", msg->length);
   //  rt_kprintf("msg buf=%08x \n", *(rt_uint8_t *)(msg->send_buf));
    
    /* take CS */
    if(msg->cs_take)
    {
        spiIoctl(0, SPI_IOC_ENABLE_SS, SPI_SS_SS0, 0);
    }

    while(len--)
    {
        if(recv_buf != RT_NULL)
        {
            *(recv_buf++) = nuc97x_transmit_byte(0, *(send_buf++));
        }
        else
        {
            nuc97x_transmit_byte(0, *(send_buf++));
        }
    }
    
    /* release CS */
    if(msg->cs_release)
    {
        spiIoctl(0, SPI_IOC_DISABLE_SS, SPI_SS_SS0, 0);
    }

    return msg->length;
}

static struct rt_spi_bus       _g_nuc972_spi0_bus;
static const struct rt_spi_ops _g_nuc97x_spi_ops =
{
    .configure = nuc97x_spi_configure,
    .xfer      = nuc97x_spi_xfer,
};

int rt_hw_spi_init(void)
{
    rt_int32_t result;
    
    struct rt_spi_bus *spi_bus = RT_NULL;
    
    /* Configure multi function pins to SPI0 */
    outpw(REG_SYS_GPB_MFPL, (inpw(REG_SYS_GPB_MFPL) & ~0xff000000) | 0xBB000000);
    outpw(REG_SYS_GPB_MFPH, (inpw(REG_SYS_GPB_MFPH) & ~0xff) | 0xBB);

    spiInit(0);
    spiOpen(0);

    // set spi interface speed to 2MHz
    spiIoctl(0, SPI_IOC_SET_SPEED, 2000000, 0);
    // set transfer length to 8-bit
    spiIoctl(0, SPI_IOC_SET_TX_BITLEN, 8, 0);
    // set transfer mode to mode-0
    spiIoctl(0, SPI_IOC_SET_MODE, 0, 0);
    
    // spiIoctl(0, SPI_IOC_SET_LSB_MSB, SPI_LSB, 0);
    
    /* register spi bus */
    spi_bus = &_g_nuc972_spi0_bus;
    spi_bus->parent.user_data = NULL;

    result = rt_spi_bus_register(spi_bus, "spi0", &_g_nuc97x_spi_ops);
    if(result != RT_EOK)
    {
        rt_kprintf("spi bus register error..\n");
        
        return result;
    }
    
    rt_kprintf("spi bus initialization done!\n");
    
    return RT_EOK;
}
#endif


