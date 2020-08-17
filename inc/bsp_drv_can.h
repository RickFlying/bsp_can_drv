#ifndef __BSP_DRV_CAN_H
#define __BSP_DRV_CAN_H


#include "stm32f4xx.h"
#include <board.h>
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
#include "can_app.h"

#define bsp_can1       "CAN1"
#define bsp_can2       "CAN2"

#define USING_CAN1

#define rx_fifo_size   16  // one frame is 16Byte

#define flag_StdID     0
#define flag_ExtID     1


#define LIST_MOD       CAN_FILTERMODE_IDLIST
#define MASK_MOD       CAN_FILTERMODE_IDMASK
#define Scale_32bit    CAN_FILTERSCALE_32BIT
#define Scale_16bit    CAN_FILTERSCALE_16BIT


#define CAN_CMD_SET_FILTER       0x11
#define CAN_CMD_SET_BAUD         0x12
#define CAN_CMD_SET_MODE         0x13
#define CAN_CMD_SET_INT          0x14


#define BSP_CAN_STDID  0
#define BSP_CAN_EXTID  1
#define BSP_CAN_DTR    0
#define BSP_CAN_RTR    1


#define BS1SHIFT       16
#define BS2SHIFT       20
#define RRESCLSHIFT    0
#define SJWSHIFT       24
#define BS1MASK        ( (0x0F) << BS1SHIFT )
#define BS2MASK        ( (0x07) << BS2SHIFT )
#define RRESCLMASK     ( 0x3FF  << RRESCLSHIFT )
#define SJWMASK        ( 0x3    << SJWSHIFT )


#define CAN_EVENT_RX_IND         0x01    /* Rx indication */
#define CAN_EVENT_RX_TIMEOUT     0x02    /* Rx timeout    */
#define CAN_EVENT_RXOF_IND       0x03    /* Rx overflow */
#define CAN_EVENT_TX_DONE        0x04    /* Tx complete   */
#define CAN_EVENT_TX_FAIL        0x05    /* Tx complete   */


#define SEND_OK       0X00
#define SEND_ERR      0X0F  



enum CANBAUD
{
    CAN1MBaud   = 1000UL * 1000,/* 1 MBit/sec   */
    CAN800kBaud = 1000UL * 800, /* 800 kBit/sec */
    CAN500kBaud = 1000UL * 500, /* 500 kBit/sec */
    CAN250kBaud = 1000UL * 250, /* 250 kBit/sec */
    CAN125kBaud = 1000UL * 125, /* 125 kBit/sec */
    CAN100kBaud = 1000UL * 100, /* 100 kBit/sec */
    CAN50kBaud  = 1000UL * 50,  /* 50 kBit/sec  */
    CAN20kBaud  = 1000UL * 20,  /* 20 kBit/sec  */
    CAN10kBaud  = 1000UL * 10   /* 10 kBit/sec  */
};



struct soft_rx_fifo
{
    /* software fifo */
    rt_uint8_t *buffer;

    rt_uint16_t put_index, get_index;

    rt_bool_t is_full;
};




struct soft_tx_fifo
{
    struct rt_can_sndbxinx_list *buffer;
    struct rt_semaphore sem;
    struct rt_list_node freelist;
};





struct rt_can_msg
{
    rt_uint32_t id  : 29;
    rt_uint32_t ide : 1;
    rt_uint32_t rtr : 1;
    rt_uint32_t rsv : 1;
    rt_uint32_t len : 8;
    rt_uint32_t priv : 8;
    rt_uint32_t hdr : 8;
    rt_uint8_t data[8];
};



struct drv_can_baud_cfg
{
    rt_uint32_t baud_rate;
    rt_uint32_t confdata;
};




struct drv_can_mod_cfg
{
    const char *name;
	rt_uint32_t baud_rate;
    rt_uint32_t mode     ;
};





struct drv_filter_para
{
	rt_uint8_t   bank;
	rt_uint8_t   filter_mod;
	rt_uint8_t   filter_scl;
	rt_uint32_t  ID_para1;
	rt_uint32_t  ID_para2;
	rt_uint32_t  ID_para3;
	rt_uint32_t  ID_para4;
};




typedef struct __drv_can_filter_cfg
{
	rt_uint8_t   bank_count;           
	rt_uint8_t   frame_mod;            
	struct drv_filter_para   *filter_para;  
}drv_can_filter_cfg;




typedef struct drv_obj_can *drv_obj_can_t; 
struct drv_obj_can
{
	CAN_HandleTypeDef hcan;   
	rt_err_t (*rx_indicate)(drv_obj_can_t p_dev_can, rt_size_t size);
    rt_err_t (*tx_complete)(drv_obj_can_t p_dev_can, rt_err_t state);
	void* rx_data;
	void* tx_data;
};


struct drv_obj_can* drv_can_open(const char *name );
rt_err_t drv_can_close(struct drv_obj_can *dev);
rt_err_t drv_can_read(struct drv_obj_can *dev, rt_off_t pos, void *temp, rt_size_t length);
rt_err_t drv_can_write(struct drv_obj_can *dev,struct rt_can_msg *txmsg);
rt_err_t drv_can_set_rx_indicate(struct drv_obj_can *dev_can,rt_err_t (*rx_ind)(drv_obj_can_t p_dev_can, rt_size_t size));
rt_err_t drv_can_set_tx_complete(struct drv_obj_can *dev_can,rt_err_t (*tx_ind)(drv_obj_can_t p_dev_can, rt_err_t state));
rt_err_t drv_can_control(struct drv_obj_can *dev, int cmd, void *arg);
rt_err_t drv_can_run(struct drv_obj_can *dev);



#endif

