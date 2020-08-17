#include "bsp_drv_can.h"


static const struct drv_can_baud_cfg can_baud_rate_tab[] =
{
    {CAN1MBaud,   (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 3)  },
    {CAN800kBaud, (CAN_SJW_2TQ | CAN_BS1_8TQ | CAN_BS2_5TQ | 4)  },
    {CAN500kBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 6)  },
    {CAN250kBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 12) },
    {CAN125kBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 24) },
    {CAN100kBaud, (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 30) },
    {CAN50kBaud,  (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 60) },
    {CAN20kBaud,  (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 150)},
    {CAN10kBaud,  (CAN_SJW_2TQ | CAN_BS1_9TQ | CAN_BS2_5TQ | 300)}
};

#define BAUD_DATA(TYPE,NO)    ((can_baud_rate_tab[NO].confdata & TYPE##MASK))



static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index, default_index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    default_index = len;

    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;

        if (can_baud_rate_tab[index].baud_rate == 1000UL * 250)
            default_index = index;
    }

    if (default_index != len)
        return default_index;

    return 0;
}



static rt_err_t bsp_can_mod_config(CAN_HandleTypeDef *hcan, struct drv_can_mod_cfg *cfg)
{
    rt_uint32_t baud_index;  
	
	if(rt_strncmp(cfg->name, bsp_can1, 8) == 0)
	{
		hcan->Instance = CAN1;
	}
	else if(rt_strncmp(cfg->name, bsp_can2, 8) == 0)
	{
		hcan->Instance = CAN2;
	}
	hcan->Init.TransmitFifoPriority = ENABLE;
    hcan->Init.AutoBusOff           = ENABLE;
    hcan->Init.AutoWakeUp           = DISABLE;
    hcan->Init.AutoRetransmission   = DISABLE;
    hcan->Init.ReceiveFifoLocked    = DISABLE;
    hcan->Init.TimeTriggeredMode    = DISABLE;

    switch (cfg->mode)
    {
		case CAN_MODE_NORMAL:
			hcan->Init.Mode = CAN_MODE_NORMAL;
			break;
		case CAN_MODE_SILENT:
			hcan->Init.Mode = CAN_MODE_SILENT;
			break;
		case CAN_MODE_LOOPBACK:
			hcan->Init.Mode = CAN_MODE_LOOPBACK;
			break;
		case CAN_MODE_SILENT_LOOPBACK:
			hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK;
			break;
		default:
			break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);

	hcan->Init.SyncJumpWidth =  BAUD_DATA(SJW,    baud_index);
    hcan->Init.TimeSeg1      =  BAUD_DATA(BS1,    baud_index);
    hcan->Init.TimeSeg2      =  BAUD_DATA(BS2,    baud_index);
    hcan->Init.Prescaler     =  BAUD_DATA(RRESCL, baud_index);
	
	if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        return RT_ERROR;
    }

    return RT_EOK;
}






rt_err_t bsp_can_filter_config(CAN_HandleTypeDef *hcan, drv_can_filter_cfg *hcfg)
{
	CAN_FilterTypeDef  FilterConfig;
	rt_uint8_t i=0;
	
	if(hcfg->frame_mod==flag_StdID)     
	{
		for(i=0;i<hcfg->bank_count;i++)     
		{                                   
			FilterConfig.FilterBank  =  hcfg->filter_para[i].bank;
			FilterConfig.FilterMode  =  hcfg->filter_para[i].filter_mod;
			FilterConfig.FilterScale =  hcfg->filter_para[i].filter_scl;
			
			if(FilterConfig.FilterMode==LIST_MOD)
			{
				if(FilterConfig.FilterScale==Scale_32bit)
				{
					FilterConfig.FilterIdHigh  = hcfg->filter_para[i].ID_para1<<5;    
					FilterConfig.FilterIdLow   = 0|CAN_ID_STD;         

					FilterConfig.FilterMaskIdHigh  =  hcfg->filter_para[i].ID_para2<<5;     
					FilterConfig.FilterMaskIdLow   =  0|CAN_ID_STD;   
				}
				else if(FilterConfig.FilterScale==Scale_16bit)
				{
					FilterConfig.FilterIdHigh  = hcfg->filter_para[i].ID_para1<<5;    
					FilterConfig.FilterIdLow   = hcfg->filter_para[i].ID_para2<<5;         

					FilterConfig.FilterMaskIdHigh  =  hcfg->filter_para[i].ID_para3<<5;     
					FilterConfig.FilterMaskIdLow   =  hcfg->filter_para[i].ID_para4<<5;
				}
			}
			else if(FilterConfig.FilterMode==MASK_MOD)
			{
				if(FilterConfig.FilterScale==Scale_32bit)
				{
					FilterConfig.FilterIdHigh  = hcfg->filter_para[i].ID_para1<<5;    
					FilterConfig.FilterIdLow   = 0;         

					FilterConfig.FilterMaskIdHigh  =  hcfg->filter_para[i].ID_para2<<5;     
					FilterConfig.FilterMaskIdLow   =  0x0006;  //0|0x02
				}
				else if(FilterConfig.FilterScale==Scale_16bit)
				{
					FilterConfig.FilterIdHigh      =  hcfg->filter_para[i].ID_para1<<5;    
					FilterConfig.FilterMaskIdHigh  =  hcfg->filter_para[i].ID_para2<<5;
					
					FilterConfig.FilterIdLow       =  hcfg->filter_para[i].ID_para3<<5;      
					FilterConfig.FilterMaskIdLow   =  hcfg->filter_para[i].ID_para4<<5;
				}
			}  

			FilterConfig.FilterFIFOAssignment    =  CAN_RX_FIFO0;       
			FilterConfig.FilterActivation        =  ENABLE;
			FilterConfig.SlaveStartFilterBank    =  14;	
		
			if(HAL_CAN_ConfigFilter(hcan, &FilterConfig)!=HAL_OK)
			{
				return RT_ERROR;
			}
		}
	}
	else if(hcfg->frame_mod==flag_ExtID)    
	{
		
	}
	
	return RT_EOK;
}





rt_err_t bsp_get_can_msg(CAN_HandleTypeDef *hcan, void* buf, rt_uint32_t fifo_num)
{
	HAL_StatusTypeDef   status;
	CAN_RxHeaderTypeDef rxheader = {0};
	struct rt_can_msg *pmsg=(struct rt_can_msg *) buf;
	
	status=HAL_CAN_GetRxMessage(hcan, fifo_num, &rxheader, pmsg->data);  //hal get data
	if(status!=HAL_OK)
	{
		return RT_ERROR;
	}
		
	if(rxheader.IDE == CAN_ID_STD)
    {
        pmsg->ide = BSP_CAN_STDID;
        pmsg->id = rxheader.StdId;
    }
    else
    {
        pmsg->ide = BSP_CAN_EXTID;
        pmsg->id = rxheader.ExtId;
    }

    if(rxheader.RTR == CAN_RTR_DATA)
    {
        pmsg->rtr = BSP_CAN_DTR;
    }
    else
    {
        pmsg->rtr = BSP_CAN_RTR;
    }

    pmsg->len = rxheader.DLC;
    pmsg->hdr = rxheader.FilterMatchIndex;
	
	return RT_EOK;
}



rt_err_t bsp_send_can_msg(CAN_HandleTypeDef *hcan, struct rt_can_msg *txmsg)
{
    CAN_TxHeaderTypeDef	 TxHeader;
	rt_uint32_t  TxMailbox;                
	
	if (txmsg->ide == BSP_CAN_STDID)
    {
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.StdId = txmsg->id;
    }
    else
    {
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.ExtId = txmsg->id;
    }
    
    if (txmsg->rtr == BSP_CAN_DTR)
    {
        TxHeader.RTR = CAN_RTR_DATA;
    }
    else
    {
        TxHeader.RTR = CAN_RTR_REMOTE;
    }
	
	TxHeader.DLC=txmsg->len;
	
	
	if(HAL_CAN_AddTxMessage(hcan, &TxHeader, txmsg->data, &TxMailbox) != HAL_OK)   
	{
		return RT_ERROR;
	}
	return RT_EOK;
}





void bsp_can_isr(struct drv_obj_can *dev, rt_uint32_t irq_flag)
{
	
	switch (irq_flag&0xff)
	{
		case CAN_EVENT_RX_IND:
					{
						rt_size_t rx_length;
						rt_base_t level;
						struct soft_rx_fifo* rx_fifo;
						struct rt_can_msg recvmsg;
						
						rx_fifo = (struct soft_rx_fifo*)dev->rx_data;
						RT_ASSERT(rx_fifo != RT_NULL);
							
						
						bsp_get_can_msg(&(dev->hcan),&(rx_fifo->buffer[rx_fifo->put_index]),irq_flag>>8); 
						
						level = rt_hw_interrupt_disable();
						
						
						rx_fifo->put_index +=sizeof(recvmsg);  

						rx_length = (rx_fifo->put_index >= rx_fifo->get_index)? (rx_fifo->put_index - rx_fifo->get_index):
									(rx_fifo_size - (rx_fifo->get_index - rx_fifo->put_index));
						

						if((dev->rx_indicate != RT_NULL) && (rx_length==rx_fifo_size))
						{			
							rx_fifo->is_full=RT_TRUE;
							dev->rx_indicate(dev, rx_length);
						}
						
						
						if(rx_fifo->put_index >= rx_fifo_size) 
						{	
							rx_fifo->put_index = 0;
						}
						
						rt_hw_interrupt_enable(level);
						
						break;
					}	
		case CAN_EVENT_RX_TIMEOUT:
					{
						
						break;
					}
			
		case CAN_EVENT_RXOF_IND:
					{
						
						break;
					}				
		case CAN_EVENT_TX_DONE:
		case CAN_EVENT_TX_FAIL:
					{
						rt_uint32_t can_fifo;
						can_fifo = irq_flag >> 8;
						
						if ((irq_flag & 0xff) == CAN_EVENT_TX_DONE)
						{
							dev->tx_complete(dev, SEND_OK|can_fifo<<8);
						}
						else
						{
							dev->tx_complete(dev, SEND_ERR|can_fifo<<8);
						}
						break;
					}
		default:
			    break;
	}
}







void bsp_can_rx_irq(struct drv_obj_can *dev,rt_uint32_t fifo)
{   
    switch (fifo)
    {
		case CAN_RX_FIFO0:
				if (HAL_CAN_GetRxFifoFillLevel(&dev->hcan, CAN_RX_FIFO0) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING))
				{
					bsp_can_isr(dev,CAN_EVENT_RX_IND|fifo<<8);
				}
				/* Check FULL flag for FIFO0 */
				if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_FF0) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO0_FULL))
				{
					/*add isr entry*/
					/* Clear FIFO0 FULL Flag */
					__HAL_CAN_CLEAR_FLAG(&dev->hcan, CAN_FLAG_FF0);
				}
				
				/* Check Overrun flag for FIFO0 */
				if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_FOV0) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO0_OVERRUN))
				{
					/*add isr entry*/
					/* Clear FIFO0 Overrun Flag */
					__HAL_CAN_CLEAR_FLAG(&dev->hcan, CAN_FLAG_FOV0);
				}
				break;
		
		case CAN_RX_FIFO1:
				if (HAL_CAN_GetRxFifoFillLevel(&dev->hcan, CAN_RX_FIFO1) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO1_MSG_PENDING))
				{
                     /*add isr entry*/
				}
				/* Check FULL flag for FIFO1 */
				if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_FF1) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO1_FULL))
				{
					/*add isr entry*/
					/* Clear FIFO1 FULL Flag */
					__HAL_CAN_CLEAR_FLAG(&dev->hcan, CAN_FLAG_FF1);
				}
				
				/* Check Overrun flag for FIFO1 */
				if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_FOV1) 
					&& __HAL_CAN_GET_IT_SOURCE(&dev->hcan, CAN_IT_RX_FIFO1_OVERRUN))
				{
					/*add isr entry*/
					/* Clear FIFO1 Overrun Flag */
					__HAL_CAN_CLEAR_FLAG(&dev->hcan, CAN_FLAG_FOV1);

				}
				break;
		default:
	            break;
    }
}	





void bsp_can_tx_irq(struct drv_obj_can *dev)
{
	if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_RQCP0))
    {
        if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_TXOK0))
        {
			bsp_can_isr(dev,CAN_EVENT_TX_DONE);
        }
        else
        {
			bsp_can_isr(dev,CAN_EVENT_TX_FAIL);
        }
        SET_BIT((&dev->hcan)->Instance->TSR, CAN_TSR_RQCP0);
    }
    else if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_RQCP1))
    {
        if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_TXOK1))
        {
			/*add isr entry*/
        }
        else
        {
			/*add isr entry*/
        }
        SET_BIT((&dev->hcan)->Instance->TSR, CAN_TSR_RQCP1);
    }
    else if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_RQCP2))
    {
        if (__HAL_CAN_GET_FLAG(&dev->hcan, CAN_FLAG_TXOK2))
        {
			/*add isr entry*/
        }
        else
        {
			/*add isr entry*/
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT((&dev->hcan)->Instance->TSR, CAN_TSR_RQCP2);
    }
}





#ifdef USING_CAN1

struct drv_obj_can   dev_can1;



void CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    bsp_can_rx_irq(&dev_can1, CAN_RX_FIFO0);
    rt_interrupt_leave();
}



void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
	bsp_can_rx_irq(&dev_can1, CAN_RX_FIFO1);
    rt_interrupt_leave();
}



void CAN1_TX_IRQHandler(void)
{
	rt_interrupt_enter();
    bsp_can_tx_irq(&dev_can1);
	rt_interrupt_leave();
}

#endif




#ifdef USING_CAN2

/* add can2 code */

#endif




struct drv_obj_can*  drv_can_open(const char *name )
{
	struct soft_rx_fifo*    rx_fifo;
	struct drv_can_mod_cfg  cfg;
	
	struct drv_obj_can*  dev=&dev_can1;
	
	if(rt_strncmp(name, bsp_can1, 8) == 0)
	{
		cfg.name=bsp_can1;
	}
	else if(rt_strncmp(name, bsp_can2, 8) == 0)
	{
		cfg.name=bsp_can2;
	}
	else
	{
		rt_kprintf("device name err!\n");
	}
	
	if(bsp_can_mod_config(&dev->hcan, &cfg)!=RT_EOK)
	{
		rt_kprintf("cfg dev name filed! \n");
	}

	
	rx_fifo = (struct soft_rx_fifo*)rt_malloc(sizeof(struct soft_rx_fifo) +rx_fifo_size);
	
	RT_ASSERT(rx_fifo != RT_NULL);
	
	rx_fifo->buffer = (rt_uint8_t*) (rx_fifo + 1);
	rt_memset(rx_fifo->buffer, 0, rx_fifo_size);
	rx_fifo->put_index = 0;
	rx_fifo->get_index = 0;
	rx_fifo->is_full = RT_FALSE;

	dev->rx_data = rx_fifo;
	
	return dev;
  
}




rt_err_t drv_can_close(struct drv_obj_can *dev)
{
	RT_ASSERT(dev!=RT_NULL);
	
	if(HAL_CAN_DeInit(&dev->hcan)!=HAL_OK)
	{
		return RT_ERROR;
	}
	
	rt_free(dev->rx_data);        
	
	rt_memset(dev, 0, sizeof(struct drv_obj_can));
	
	return RT_EOK; 
}




rt_err_t drv_can_read(struct drv_obj_can *dev, rt_off_t pos, void *temp, rt_size_t length)
{
	int size;
    struct soft_rx_fifo* rx_fifo;
    rt_base_t level;
    RT_ASSERT(dev != RT_NULL);
    size = length;   

    rx_fifo = (struct soft_rx_fifo*) dev->rx_data;
    RT_ASSERT(rx_fifo != RT_NULL);

    if(rx_fifo->is_full==RT_FALSE)
	{
		return 0;
	}
	else if(rx_fifo->is_full==RT_TRUE)  
	{
		level = rt_hw_interrupt_disable();
		while (length)
		{
			rt_memcpy(temp,&(rx_fifo->buffer[rx_fifo->get_index]),length);
			
			rx_fifo->get_index +=length;
			
			if (rx_fifo->get_index >= rx_fifo_size) 
			{
				rx_fifo->get_index = 0;
			}
			
			length=0;
		}
		
		rx_fifo->is_full=RT_FALSE;
		rt_hw_interrupt_enable(level);
		
	}
	return size; 
}





rt_err_t drv_can_write(struct drv_obj_can *dev,struct rt_can_msg *txmsg)
{
	bsp_send_can_msg(&dev->hcan,txmsg);
}





rt_err_t drv_can_set_rx_indicate(struct drv_obj_can *dev_can,
	                             rt_err_t (*rx_ind)(drv_obj_can_t p_dev_can, rt_size_t size))
{
	RT_ASSERT(dev_can != RT_NULL);
    dev_can->rx_indicate = rx_ind;

    return RT_EOK;
}





rt_err_t drv_can_set_tx_complete(struct drv_obj_can *dev_can,
	                             rt_err_t (*tx_ind)(drv_obj_can_t p_dev_can, rt_err_t state))
{
	RT_ASSERT(dev_can != RT_NULL);
    dev_can->tx_complete = tx_ind;

    return RT_EOK;
}





rt_err_t drv_can_control(struct drv_obj_can *dev, int cmd, void *arg)
{
    rt_uint32_t argval;
	static struct drv_can_mod_cfg cfg;
	
	switch(cmd)
    {
		case CAN_CMD_SET_BAUD:
					argval = (rt_uint32_t) arg;
					
					if (argval != CAN1MBaud   &&
						argval != CAN800kBaud &&
						argval != CAN500kBaud &&
						argval != CAN250kBaud &&
						argval != CAN125kBaud &&
						argval != CAN100kBaud &&
						argval != CAN50kBaud  &&
						argval != CAN20kBaud  &&
						argval != CAN10kBaud)
					{
						return RT_ERROR;
					}
													
					cfg.baud_rate=argval;
					if(bsp_can_mod_config(&dev->hcan, &cfg)!=RT_EOK)
					{
						rt_kprintf("cfg baud filed! \n");
					}
					
					break;
						
		case CAN_CMD_SET_MODE:
					argval = (rt_uint32_t) arg;
					
					if (argval != CAN_MODE_NORMAL &&
						argval != CAN_MODE_LOOPBACK &&
						argval != CAN_MODE_SILENT &&
						argval != CAN_MODE_SILENT_LOOPBACK)
					{
						return RT_ERROR;
					}

					cfg.mode=argval;
					if(bsp_can_mod_config(&dev->hcan, &cfg)!=RT_EOK)
					{
						rt_kprintf("cfg mod filed! \n");
					}

					break;
								
		case CAN_CMD_SET_FILTER:
					if (RT_NULL == arg)
					{
					   return RT_ERROR;
					}
					else
					{
						bsp_can_filter_config(&dev->hcan, (drv_can_filter_cfg*)arg);
					}
					break;
        								
		case CAN_CMD_SET_INT:
					argval = (rt_uint32_t) arg;
					if (argval == RT_DEVICE_FLAG_INT_RX)
					{
						__HAL_CAN_ENABLE_IT(&dev->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
						   
						if (dev->hcan.Instance==CAN1)
						{
							HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
							HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
						}
					}
					else if (argval == RT_DEVICE_FLAG_INT_TX)
					{
						__HAL_CAN_ENABLE_IT(&dev->hcan, CAN_IT_TX_MAILBOX_EMPTY);
						
						if (dev->hcan.Instance==CAN1)
						{
							HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
							HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
						}
					}
					break;
		default:
                break;			

    }
	
    return RT_EOK;
}






rt_err_t drv_can_run(struct drv_obj_can *dev)
{
    
	if(HAL_CAN_Start(&dev->hcan)!=HAL_OK)
	{
		return RT_ERROR; 
		
	}
	return RT_EOK; 
}




