/*******************************Copyright (c)***************************
**
**  							   SIMULATE
**
**------------------File Info-------------------------------------------
** File name:            	gps.h
** Latest Version:      	V1.0.0
** Latest modified Date:	2015/06/05
** Modified by:
** Descriptions:
**
**----------------------------------------------------------------------
** Created by:           	Zhu Zheng <happyzhull@163.com>
** Created date:         	2015/06/05
** Descriptions:
**
***********************************************************************/
#pragma once
#include "type.h"
#include "misc_usr.h"
#include "device.h"


#if 0
//GPS NMEA-0183Э����Ҫ�����ṹ�嶨��
//������Ϣ
__packed typedef struct
{
 	U8  num;		//���Ǳ��
	U8  eledeg;	//��������
	U16 azideg;	//���Ƿ�λ��
	U8  sn;		//�����
}nmea_slmsg;
//UTCʱ����Ϣ
__packed typedef struct
{
 	U16 year;	//���
	U8  month;	//�·�
	U8  date;	//����
	U8  hour; 	//Сʱ
	U8  min; 	//����
	U8  sec; 	//����
}nmea_utc_time;
//NMEA 0183 Э����������ݴ�Žṹ��
__packed typedef struct
{
 	U8 svnum;					//�ɼ�������
	nmea_slmsg slmsg[12];		//���12������
	nmea_utc_time utc;			//UTCʱ��
	u32 latitude;				//γ�� ������100000��,ʵ��Ҫ����100000
	U8 nshemi;					//��γ/��γ,N:��γ;S:��γ
	u32 longitude;			    //���� ������100000��,ʵ��Ҫ����100000
	U8 ewhemi;					//����/����,E:����;W:����
	U8 gpssta;					//GPS״̬:0,δ��λ;1,�ǲ�ֶ�λ;2,��ֶ�λ;6,���ڹ���.
 	U8 posslnum;				//���ڶ�λ��������,0~12.
 	U8 possl[12];				//���ڶ�λ�����Ǳ��
	U8 fixmode;					//��λ����:1,û�ж�λ;2,2D��λ;3,3D��λ
	U16 pdop;					//λ�þ������� 0~500,��Ӧʵ��ֵ0~50.0
	U16 hdop;					//ˮƽ�������� 0~500,��Ӧʵ��ֵ0~50.0
	U16 vdop;					//��ֱ�������� 0~500,��Ӧʵ��ֵ0~50.0

	int altitude;			 	//���θ߶�,�Ŵ���10��,ʵ�ʳ���10.��λ:0.1m
	U16 speed;					//��������,�Ŵ���1000��,ʵ�ʳ���10.��λ:0.001����/Сʱ
}nmea_msg;
////////////////////////////////////////////////////////////////////////////////////////////////////
//UBLOX NEO-6M ����(���,����,���ص�)�ṹ��
__packed typedef struct
{
 	U16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	U16 id;						//CFG CFG ID:0X0906 (С��ģʽ)
	U16 dlength;				//���ݳ��� 12/13
	u32 clearmask;				//�������������(1��Ч)
	u32 savemask;				//�����򱣴�����
	u32 loadmask;				//�������������
	U8  devicemask; 		  	//Ŀ������ѡ������	b0:BK RAM;b1:FLASH;b2,EEPROM;b4,SPI FLASH
	U8  cka;		 			//У��CK_A
	U8  ckb;			 		//У��CK_B
}_ublox_cfg_cfg;

//UBLOX NEO-6M ��Ϣ���ýṹ��
__packed typedef struct
{
 	U16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	U16 id;						//CFG MSG ID:0X0106 (С��ģʽ)
	U16 dlength;				//���ݳ��� 8
	U8  msgclass;				//��Ϣ����(F0 ����NMEA��Ϣ��ʽ)
	U8  msgid;					//��Ϣ ID
								//00,GPGGA;01,GPGLL;02,GPGSA;
								//03,GPGSV;04,GPRMC;05,GPVTG;
								//06,GPGRS;07,GPGST;08,GPZDA;
								//09,GPGBS;0A,GPDTM;0D,GPGNS;
	U8  iicset;					//IIC���������    0,�ر�;1,ʹ��.
	U8  uart1set;				//UART1�������	   0,�ر�;1,ʹ��.
	U8  uart2set;				//UART2�������	   0,�ر�;1,ʹ��.
	U8  usbset;					//USB�������	   0,�ر�;1,ʹ��.
	U8  spiset;					//SPI�������	   0,�ر�;1,ʹ��.
	U8  ncset;					//δ֪�������	   Ĭ��Ϊ1����.
 	U8  cka;			 		//У��CK_A
	U8  ckb;			    	//У��CK_B
}_ublox_cfg_msg;

//UBLOX NEO-6M UART�˿����ýṹ��
__packed typedef struct
{
 	U16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	U16 id;						//CFG PRT ID:0X0006 (С��ģʽ)
	U16 dlength;				//���ݳ��� 20
	U8  portid;					//�˿ں�,0=IIC;1=UART1;2=UART2;3=USB;4=SPI;
	U8  reserved;				//����,����Ϊ0
	U16 txready;				//TX Ready��������,Ĭ��Ϊ0
	u32 mode;					//���ڹ���ģʽ����,��żУ��,ֹͣλ,�ֽڳ��ȵȵ�����.
 	u32 baudrate;				//����������
 	U16 inprotomask;		 	//����Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	U16 outprotomask;		 	//���Э�鼤������λ  Ĭ������Ϊ0X07 0X00����.
 	U16 reserved4; 				//����,����Ϊ0
 	U16 reserved5; 				//����,����Ϊ0
 	U8  cka;			 		//У��CK_A
	U8  ckb;			    	//У��CK_B
}_ublox_cfg_prt;

//UBLOX NEO-6M ʱ���������ýṹ��
__packed typedef struct
{
 	U16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	U16 id;						//CFG TP ID:0X0706 (С��ģʽ)
	U16 dlength;				//���ݳ���
	u32 interval;				//ʱ��������,��λΪus
	u32 length;				 	//������,��λΪus
	signed char status;			//ʱ����������:1,�ߵ�ƽ��Ч;0,�ر�;-1,�͵�ƽ��Ч.
	U8 timeref;			   		//�ο�ʱ��:0,UTCʱ��;1,GPSʱ��;2,����ʱ��.
	U8 flags;					//ʱ���������ñ�־
	U8 reserved;				//����
 	signed short antdelay;	 	//������ʱ
 	signed short rfdelay;		//RF��ʱ
	signed int userdelay; 	 	//�û���ʱ
	U8 cka;						//У��CK_A
	U8 ckb;						//У��CK_B
}_ublox_cfg_tp;

//UBLOX NEO-6M ˢ���������ýṹ��
__packed typedef struct
{
 	U16 header;					//cfg header,�̶�Ϊ0X62B5(С��ģʽ)
	U16 id;						//CFG RATE ID:0X0806 (С��ģʽ)
	U16 dlength;				//���ݳ���
	U16 measrate;				//����ʱ��������λΪms�����ٲ���С��200ms��5Hz��
	U16 navrate;				//�������ʣ����ڣ����̶�Ϊ1
	U16 timeref;				//�ο�ʱ�䣺0=UTC Time��1=GPS Time��
 	U8  cka;					//У��CK_A
	U8  ckb;					//У��CK_B
}_ublox_cfg_rate;

int NMEA_Str2num(U8 *buf,U8*dx);
void GPS_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGGA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPGSA_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPRMC_Analysis(nmea_msg *gpsx,U8 *buf);
void NMEA_GPVTG_Analysis(nmea_msg *gpsx,U8 *buf);
U8 Ublox_Cfg_Cfg_Save(void);
U8 Ublox_Cfg_Msg(U8 msgid,U8 uart1set);
U8 Ublox_Cfg_Prt(u32 baudrate);
U8 Ublox_Cfg_Tp(u32 interval,u32 length,signed char status);
U8 Ublox_Cfg_Rate(U16 measrate,U8 reftime);
void Ublox_Send_Date(U8* dbuf,U16 len);
#endif



struct gps_device {
	U32 gps_uart_banud;
	struct uart_device *uart_dev;

	struct device dev;
};

struct gps_platform {
	PCSTR name;
	S32	id;
	U32 gps_uart_banud;
	struct uart_device *uart_dev;
};


struct gps_device *gps_dev_constructe(struct gps_platform *plat);
S32 gps_dev_destroy(struct gps_device *gps_dev);

/***********************************************************************
** End of file
***********************************************************************/


