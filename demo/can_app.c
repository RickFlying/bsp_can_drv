#include "can_app.h"
#include "bsp_drv_can.h"  



struct drv_filter_para  filter_para[4]=
{    
	{0, LIST_MOD, Scale_32bit, 0x321, 0x000, NULL,  NULL },
	{1, LIST_MOD, Scale_16bit, 0x123, 0x456, 0x789, 0x199},
	{2, MASK_MOD, Scale_32bit, 0x300, 0x7f0, NULL,  NULL },
	{3, MASK_MOD, Scale_16bit, 0x410, 0x7f0, 0x510, 0x7f0}
};  //配置过滤器时增加NULL判断？？？？

drv_can_filter_cfg  can1_filter_cfg={4,flag_StdID,filter_para};

static struct       rt_semaphore rx_sem;   

drv_obj_can_t       CAN1_DEV;    //can1设备对象





static rt_err_t can_rx_callback(drv_obj_can_t _dev_can, rt_size_t size)
{
	rt_kprintf("call back recv data len %d \n",size);
    
	rt_sem_release(&rx_sem);
	
    return RT_EOK;
} 




static rt_err_t can_tx_callback(drv_obj_can_t _dev_can, rt_err_t state)
{
	rt_kprintf("fifo : %d , state : %x\n",state>>8,state&0x0f);
	
    return RT_EOK;
} 




static void can_thread_entry(void *parameter)
{
	rt_uint8_t buf[8]={0};
	rt_uint8_t size;
	rt_uint8_t i=0;
	struct rt_can_msg can1_rxmsg = {0};
	struct rt_can_msg can1_txmsg = {0};
	
	can1_txmsg.id  = 0x456;             
    can1_txmsg.ide = BSP_CAN_STDID;    
    can1_txmsg.rtr = BSP_CAN_DTR;       
    can1_txmsg.len = 8;                

    can1_txmsg.data[0] = 0xF0;
    can1_txmsg.data[1] = 0xF1;
    can1_txmsg.data[2] = 0xF2;
    can1_txmsg.data[3] = 0xF3;
    can1_txmsg.data[4] = 0xF4;
    can1_txmsg.data[5] = 0xF5;
    can1_txmsg.data[6] = 0xF6;
    can1_txmsg.data[7] = 0xF7;
	
	drv_can_write(CAN1_DEV,&can1_txmsg);
	
	while(1)
	{
//		drv_can_write(CAN1_DEV,&can1_txmsg);  //自环测试用
		
		rt_sem_take(&rx_sem, RT_WAITING_FOREVER);

        size= drv_can_read(CAN1_DEV, 0, &can1_rxmsg, sizeof(can1_rxmsg));
		
		rt_kprintf("------------------------\n");
		rt_kprintf("ID   :  %x\n",can1_rxmsg.id);
		rt_kprintf("IDE  :  %x\n",can1_rxmsg.ide);
		rt_kprintf("HDR  :  %x\n",can1_rxmsg.hdr);
		rt_kprintf("DLC  :  %x\n",can1_rxmsg.len);
		rt_kprintf("DATA :  ");
		for(i=0;i<8;i++)
		{
			rt_kprintf("%x ",can1_rxmsg.data[i]);
		}	
		rt_kprintf("\n");
		rt_kprintf("------------------------\n");
		
		drv_can_write(CAN1_DEV,&can1_txmsg);  //自环测试用
		
//		rt_thread_mdelay(500);
	}
//		drv_can_close(CAN1_DEV);
}





void can1_running_init(void)
{	
    rt_uint8_t res;
	
	rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		
	CAN1_DEV=drv_can_open("CAN1");
	
	drv_can_control(CAN1_DEV,CAN_CMD_SET_BAUD,(void *)CAN500kBaud);

	drv_can_control(CAN1_DEV,CAN_CMD_SET_MODE,(void *)CAN_MODE_NORMAL); //CAN_MODE_LOOPBACK  //CAN_MODE_NORMAL

	drv_can_control(CAN1_DEV,CAN_CMD_SET_FILTER,&can1_filter_cfg);

	drv_can_control(CAN1_DEV,CAN_CMD_SET_INT,(void *)RT_DEVICE_FLAG_INT_TX);
	
    drv_can_control(CAN1_DEV,CAN_CMD_SET_INT,(void *)RT_DEVICE_FLAG_INT_RX);
	
	drv_can_set_rx_indicate(CAN1_DEV,can_rx_callback);
	
	drv_can_set_tx_complete(CAN1_DEV,can_tx_callback);
	
	res=drv_can_run(CAN1_DEV);
	if(res==RT_ERROR)
	{
		rt_kprintf("can run filed\n");
	}
	
	
	rt_thread_t can_thread = rt_thread_create("tcan", 
											  can_thread_entry,
											  RT_NULL, 
											  1024, 
	                                          15, 
	                                          5);

    if (can_thread != RT_NULL)
    {
        rt_thread_startup(can_thread);
    }
}
INIT_APP_EXPORT(can1_running_init);






