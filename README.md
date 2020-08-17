# CAN驱动说明文档

---

## 1、结构体



### 1.1、 CAN接收数据软件FIFO

```c
struct soft_rx_fifo
{
    rt_uint8_t *buffer;
    rt_uint16_t put_index, get_index;
    rt_bool_t is_full;
};
```

### 1.2、CAN发送数据软件FIFO

```c
struct soft_tx_fifo
{
    struct rt_can_sndbxinx_list *buffer;
    struct rt_semaphore sem;
    struct rt_list_node freelist;
};
```

### 1.3、 CAN消息体

```c
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
```

### 1.4、 CAN波特率参数

```c
struct drv_can_baud_cfg
{
    rt_uint32_t baud_rate;
    rt_uint32_t confdata;
};
```

### 1.5、 CAN模式配置

```c
struct drv_can_mod_cfg
{
    const char *name;
    rt_uint32_t baud_rate;
    rt_uint32_t mode;
};
```

### 1.6、 CAN过滤器参数

```c
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
```

### 1.7、 CAN过滤器配置

```c
typedef struct __drv_can_filter_cfg
{
	rt_uint8_t   bank_count;          
	rt_uint8_t   frame_mod;            
	struct drv_filter_para   *filter_para;  
}drv_can_filter_cfg;
```

### 1.8、 CAN设备对象

```c
typedef struct drv_obj_can *drv_obj_can_t;  
struct drv_obj_can
{
	CAN_HandleTypeDef hcan;    
	rt_err_t (*rx_indicate)(drv_obj_can_t p_dev_can, rt_size_t size);
    rt_err_t (*tx_complete)(drv_obj_can_t p_dev_can, rt_err_t state);
	void* rx_data;
	void* tx_data;
};
```

### 1.9、 CAN波特率枚举

```c
enum CANBAUD
{
    CAN1MBaud   = 1000UL * 1000, /* 1 MBit/sec   */
    CAN800kBaud = 1000UL * 800,  /* 800 kBit/sec */
    CAN500kBaud = 1000UL * 500,  /* 500 kBit/sec */
    CAN250kBaud = 1000UL * 250,  /* 250 kBit/sec */
    CAN125kBaud = 1000UL * 125,  /* 125 kBit/sec */
    CAN100kBaud = 1000UL * 100,  /* 100 kBit/sec */
    CAN50kBaud  = 1000UL * 50,   /* 50 kBit/sec  */
    CAN20kBaud  = 1000UL * 20,   /* 20 kBit/sec  */
    CAN10kBaud  = 1000UL * 10    /* 10 kBit/sec  */
};
```

---



## 2、宏定义



### 2.1、CAN设备名字符串

```python
#define bsp_can1     "CAN1"     
#define bsp_can2     "CAN2"
```

### 2.2、CAN通用配置

```python
#define rx_fifo_size    16  
#define flag_StdID      0
#define flag_ExtID      1
```

### 2.3、CAN过滤表配置模式

```python
#define  LIST_MOD        CAN_FILTERMODE_IDLIST
#define  MASK_MOD        CAN_FILTERMODE_IDMASK
#define  Scale_32bit     CAN_FILTERSCALE_32BIT
#define  Scale_16bit     CAN_FILTERSCALE_16BIT
```

### 2.4、CAN帧格式信息定义 

```python
#define  BSP_CAN_STDID   0
#define  BSP_CAN_EXTID   1
#define  BSP_CAN_DTR     0
#define  BSP_CAN_RTR     1
```

### 2.5、CAN模式配置指令

```python
#define  CAN_CMD_SET_FILTER     0x11
#define  CAN_CMD_SET_BAUD       0x12
#define  CAN_CMD_SET_MODE       0x13
#define  CAN_CMD_SET_INT        0x14
```

### 2.6、CAN波特率配置宏函数掩码定义

```python
#define  BS1SHIFT        16
#define  BS2SHIFT        20
#define  RRESCLSHIFT     0
#define  SJWSHIFT        24
#define  BS1MASK         ( (0x0F) << BS1SHIFT )
#define  BS2MASK         ( (0x07) << BS2SHIFT )
#define  RRESCLMASK      ( 0x3FF  << RRESCLSHIFT )
#define  SJWMASK         ( 0x3    << SJWSHIFT )
```

### 2.7、中断触发类型定义

```python
#define  CAN_EVENT_RX_IND         0x01    
#define  CAN_EVENT_RX_TIMEOUT     0x02    
#define  CAN_EVENT_RXOF_IND       0x03    
#define  CAN_EVENT_TX_DONE        0x04    
#define  CAN_EVENT_TX_FAIL        0x05    
```

### 2.8、发送中断完成状态定义

```python
#define  SEND_OK       0X00
#define  SEND_ERR      0X0F
```

---



## 4、函数



### 4.1、API操作接口



#### 4.1.1、打开CAN设备

```c
struct drv_obj_can*  drv_can_open(const char *name )
```

| 参数   | 描述                                           |
| :----- | :--------------------------------------------- |
| name   | CAN设备字符串名称 （如：“CAN1”）               |
| return | 返回  CAN设备对象的地址到用户定义的CAN对象指针 |



#### 4.1.2、配置CAN设备

```c
rt_err_t drv_can_control(struct drv_obj_can *dev, int cmd, void *arg)
```

| 参数   | 描述                             |
| :----- | :------------------------------- |
| *dev   | CAN设备对象指针                  |
| cmd    | 配置模式指令                     |
| *arg   | 配置目标参数                     |
| return | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.3、运行CAN设备

```c
rt_err_t drv_can_run(struct drv_obj_can *dev)
```

| 参数   | 描述                             |
| :----- | :------------------------------- |
| *dev   | CAN设备对象指针                  |
| return | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.4、关闭CAN设备

```c
rt_err_t drv_can_close(struct drv_obj_can *dev)
```

| 参数   | 描述                            |
| :----- | :------------------------------ |
| *dev   | CAN设备对象指针                 |
| return | 返回 RT_EOK 成功   RT_ERROR失败 |



#### 4.1.5、读取CAN设备

```c
rt_err_t drv_can_read(struct drv_obj_can *dev, rt_off_t pos, void *temp, rt_size_t length)
```

| 参数   | 描述                             |
| :----- | :------------------------------- |
| *dev   | CAN设备对象指针                  |
| pos    | 读取数据起始偏移量               |
| *temp  | 待储存数据指针                   |
| length | 读取数据长度                     |
| return | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.6、写入CAN设备

```c
rt_err_t drv_can_write(struct drv_obj_can *dev,struct rt_can_msg *txmsg)
```

| 参数   | 描述                             |
| :----- | :------------------------------- |
| *dev   | CAN设备对象指针                  |
| *txmsg | 配置目标参数                     |
| return | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.7、指定CAN接收完成回调函数

```c
rt_err_t drv_can_set_rx_indicate(struct drv_obj_can *dev,  
                                 rt_err_t (*rx_ind)(drv_obj_can_t p_dev_can, rt_size_t size))
```

| 参数      | 描述                             |
| :-------- | :------------------------------- |
| *dev      | CAN设备对象指针                  |
| (*rx_ind) | 待指定接收完成回调函数           |
| return    | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.8、指定CAN发送完成回调函数

```c
rt_err_t drv_can_set_tx_complete(struct drv_obj_can *dev,
	                             rt_err_t (*tx_ind)(drv_obj_can_t p_dev_can, rt_err_t state))
```

| 参数      | 描述                             |
| :-------- | :------------------------------- |
| *dev      | CAN设备对象指针                  |
| (*tx_ind) | 待指定发送完成回调函数           |
| return    | 返回  RT_EOK 成功   RT_ERROR失败 |



#### 4.1.9、获取波特率参数列表索引值

```c
static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
```

| 参数   | 描述           |
| :----- | :------------- |
| baud   | 波特率枚举参数 |
| return | 返回 无        |





### 4.2 底层配置与驱动函数



#### 4.2.1、CAN模式配置

```c
static rt_err_t bsp_can_mod_config(CAN_HandleTypeDef *hcan, struct drv_can_mod_cfg *cfg)
```

| 参数   | 描述                            |
| :----- | :------------------------------ |
| *hcan  | HAL库CAN句柄指针                |
| *cfg   | CAN模式配置结构体指针           |
| return | 返回 RT_EOK 成功   RT_ERROR失败 |



#### 4.1.2、CAN过滤器配置

```c
rt_err_t bsp_can_filter_config(CAN_HandleTypeDef *hcan, drv_can_filter_cfg *cfg)
```

| 参数   | 描述                            |
| :----- | :------------------------------ |
| *hcan  | HAL库CAN句柄指针                |
| *cfg   | CAN过滤器配置结构体指针         |
| return | 返回 RT_EOK 成功   RT_ERROR失败 |



#### 4.1.3、CAN接收消息

```c
rt_err_t bsp_get_can_msg(CAN_HandleTypeDef *hcan, void* buf, rt_uint32_t fifo_num)
```

| 参数     | 描述                                               |
| :------- | :------------------------------------------------- |
| *hcan    | HAL库CAN句柄指针                                   |
| *buf     | 待接收存储数据指针                                 |
| fifo_num | FIFO标号（对应STM32中bxCAN外设的两个硬件三级FIFO） |
| return   | 返回 RT_EOK 成功   RT_ERROR失败                    |



#### 4.1.4、CAN发送消息

```c
rt_err_t bsp_push_can_msg(CAN_HandleTypeDef *hcan, struct rt_can_msg *txmsg)
```

| 参数   | 描述                            |
| :----- | :------------------------------ |
| *hcan  | HAL库CAN句柄指针                |
| *txmsg | CAN过滤器配置结构体指针         |
| return | 返回 RT_EOK 成功   RT_ERROR失败 |



#### 4.1.5、CAN接收中断响应

```c
void bsp_can_rx_irq(struct drv_obj_can *dev,rt_uint32_t fifo)
```

| 参数   | 描述                                               |
| :----- | :------------------------------------------------- |
| *dev   | CAN设备对象指针                                    |
| fifo   | FIFO标号（对应STM32中bxCAN外设的两个硬件三级FIFO） |
| return | 返回 RT_EOK 成功   RT_ERROR失败                    |



#### 4.1.6、CAN发送中断响应

```c
void bsp_can_tx_irq(struct drv_obj_can *dev)
```

| 参数   | 描述            |
| :----- | :-------------- |
| *dev   | CAN设备对象指针 |
| return | 返回 无         |



#### 4.1.7、CAN中断服务

```c
void bsp_can_isr(struct drv_obj_can *dev, rt_uint32_t irq_flag)
```

| 参数     | 描述            |
| :------- | :-------------- |
| *dev     | CAN设备对象指针 |
| irq_flag | 中断响应类型    |
| return   | 返回 无         |



---



## 5、说明&注意事项

---

* 本驱动针对RT-Thread 4.0.2嵌入式实时操作系统编写，数据类型沿袭RT-Thread 4.0.2 中的类型定义 。
* 底层依赖STM32官方HAL库。
* CAN1已在STM32F407，STM32F429，以及GD32F450平台上验证成功（CAN2类似功能后续验证）。
* 目前驱动覆盖功能包括CAN常规中断接收与中断发送和轮询发送功能，FIFO全满中断和溢出中断功能框架已覆盖，根据需要自行增加服务函数处理流程。
* 目前CAN过滤器配置只覆盖标准帧格式下的配置，拓展帧对应的配置需要自行增加相应代码配置。
* 目前在RT-Thread上使用，没有对接RT的设备管理框架。

## 6、开源协议&联系方式

* 开源：采用 MIT开源协议，细节请阅读项目中的 LICENSE 文件内容。

* 维护：Rick Grimes   1050434095@qq.com