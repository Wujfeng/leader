/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/31
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "interrupt.h"
#include "device.h"
#include "dma.h"
#include "i2c.h"

#include "packet.h"

//bit0-bit1 xyz�Ƿ�ʹ��ƫѹ,Ĭ��Ϊ0��������
//bit2-bit4 �����������, 110Ϊ���75HZ 100Ϊ15HZ ��С000 0.75HZ
//bit5-bit5ÿ�β���ƽ���� 11Ϊ8�� 00Ϊһ��
#define ADDR_CONFIG_A 	0x00 	// r/w

//bit7-bit5�ų����� ����Խ��,����ԽС Ĭ��001
#define ADDR_CONFIG_B 	0x01 	// r/w

//bit0-bit1 ģʽ���� 00Ϊ�������� 01Ϊ��һ����
#define ADDR_MODE 		0x02 	// r/w
#define ADDR_DATA_X_MSB 	0x03	// r
#define ADDR_DATA_X_LSB 	0x04	// r
#define ADDR_DATA_Z_MSB 	0x05	// r
#define ADDR_DATA_Z_LSB 	0x06	// r
#define ADDR_DATA_Y_MSB 	0x07	// r
#define ADDR_DATA_Y_LSB 	0x08	// r

//bit1 ���ݸ���ʱ��λ�Զ�����,�ȴ��û���ȡ,��ȡ��һ���ʱ���ֹ���ݸı�
//bit0 �����Ѿ�׼���õȴ���ȡ��,DRDY����Ҳ����
#define ADDR_STATUS 		0x09	// r

//����ʶ��Ĵ���,���ڼ��оƬ������
#define ADDR_ID_A	0x0A	// r
#define ADDR_ID_B	0x0B	// r
#define ADDR_ID_C	0x0C	// r

//����ʶ��Ĵ�����Ĭ��ֵ
#define ID_A_WHO_AM_I			'H' //0x48
#define ID_B_WHO_AM_I			'4' //0x34
#define ID_C_WHO_AM_I			'3' //0x33


#define HMC5883_SLAVE_ADDRESS 0x3C //д��ַ,����ַ+1


//HMC5883 ��ʼ���궨��
#define HMC_DEFAULT_CONFIGA_VALUE       0x78     //75hz 8������ ��������
#define HMC_DEFAULT_CONFIGB_VALUE       0x00     //+-0.88GA����
#define HMC_DEFAULT_MODE_VALUE          0x00     //��������ģʽ


namespace driver {

class hmc5883 : public device, public interrupt
{
public:
    hmc5883(PCSTR name, s32 id);
    ~hmc5883(void);

public:
	u8 _slave_addr;
	i2c *_i2c;

public:
    s32 probe(i2c *pi2c, u8 slave_addr);
    s32 remove(void);


public:
	s32 init(void);
	s32 read_raw(void);

	s32 read_reg8(u8 reg);
	s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 *buf, u8 len);
	s32 write_reg(u8 reg, u8 *buf, u8 len);
};
}

