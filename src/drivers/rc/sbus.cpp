/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/22
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "sbus.h"
#include "core.h"

namespace driver {

sbus::sbus(PCSTR devname, s32 devid) :
	device(devname, devid)
{

}

sbus::~sbus(void)
{

}

s32 sbus::probe(void)
{
    _tx = new gpio("sbus_rx[gpio-38]", 38);
    _tx->probe();
    _tx->set_direction_output();

    while (1) {
        sbus::write_byte(0xFF);
    }

	return 0;
}


s32 sbus::remove(void)
{
	return 0;
}


s32 sbus::init(void)
{

}

s32 sbus::reset(void)
{

}

void sbus::measure(void)
{

}

void sbus::write_byte(s8 c)
{
    timestamp_t timeout = 0;
    timestamp_t inc = 104 * core::s_freq_mhz;
    //������ʼλ
    
    timeout = core::get_timestamp() + inc;
    _tx->set_value(VLOW);
    while (time_after(timeout, core::get_timestamp()));

    //����8λ����λ
    u8 i = 0;
loop:
    timeout = core::get_timestamp() + inc;
    _tx->set_value((c>>i) & 0x01);//�ȴ���λ
    if (i < 8) {
        i++;
        goto loop;
    }
    while (time_after(timeout, core::get_timestamp()));

    //����У��λ(��)

    //���ͽ���λ
    timeout = core::get_timestamp() + inc;
    _tx->set_value(VHIGH);
    while (time_after(timeout, core::get_timestamp()));
}

#if 0
//�Ӵ��ڶ�һ���ֽ�
uchar RByte(void)

{

     uchar Output=0;

     uchar i=8;

     uchar temp=RDDYN;

     //����8λ����λ

Delay2cp(RDDYN*1.5);          //�˴�ע�⣬�ȹ���ʼλ

     while(i--)

     {

         Output >>=1;

         if(RXD) Output   |=0x80;      //���յ�λ

         Delay2cp(35);              //(96-26)/2��ѭ����ռ��26��ָ������

     }

     while(--temp)                     //��ָ����ʱ������Ѱ����λ��

     {

         Delay2cp(1);

         if(RXD)break;              //�յ�����λ���˳�

     }

     return Output;

}


void sbus::read_byte(void)
{

}
#endif
}

/***********************************************************************
** End of file
***********************************************************************/
