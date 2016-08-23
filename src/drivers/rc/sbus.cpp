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

//һ��Э��˵����
//��������Ϊ������100kbps��8λ���ݣ�żУ��(even)��2λֹͣλ�������ء�
//����https://mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/˵����S-bus֡��ʽ��
//ÿ֡25���ֽڣ���������˳�����У�
//[startbyte] [data1] [data2] .... [data22] [flags][endbyte]
//��ʼ�ֽ�startbyte = 11110000b (0xF0)����ʵ������STM32����˵ARM�ˣ��յ�����0x0F��
//�м�22���ֽھ���16��ͨ���������ˣ�Ϊʲô��16��ͨ������Ϊ22x8=11x16��ÿ��ͨ����11bit��ʾ����Χ��0-2047�����ſ�����ͼ��

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
#if 0
    _rx = new gpio("sbus_rx[gpio-39]", 38);
    _rx->probe();
    _rx->set_direction_input();
    s8 c1 = 0;
    s8 c2 = 0;
    while (1) {
        c1 = sbus::read_byte();
        c2 = sbus::read_byte();
        core::mdelay(200);
    }
#else
    _tx = new gpio("sbus_tx[gpio-39]", 38);
    _tx->probe();
    _tx->set_direction_output();
    while (1) {
        sbus::write_byte(0xA5);
        core::mdelay(500);
    }
#endif
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
    timestamp_t timestamp_end = 0;
    timestamp_t inc = core::convert_us_to_timestamp(DEFAULT_BAUD_DELAY_US) - 12*26;

    //������ʼλ
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value(VLOW);
    while (time_after(timestamp_end, core::get_timestamp()));

    //����8λ����λ
#if 0
    u8 i = 0;
loop:
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>i) & 0x01);//�ȴ���λ
    if (i < 8) {
        i++;
        while (time_after(timestamp_end, core::get_timestamp()));
        goto loop;
    }
#elif 0
    for (u8 i = 0; i < 8; i++) {
        timestamp_end = core::get_timestamp() + inc;
        _tx->set_value((c>>i) & 0x01);//�ȴ���λ
        while (time_after(timestamp_end, core::get_timestamp()));
    }
#else
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>0) & 0x01);//��0λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>1) & 0x01);//��1λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>2) & 0x01);//��2λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>3) & 0x01);//��3λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>4) & 0x01);//��4λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>5) & 0x01);//��5λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>6) & 0x01);//��6λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value((c>>7) & 0x01);//��7λ
    while (time_after(timestamp_end, core::get_timestamp()));
#endif

    //����У��λ(��)

    //���ͽ���λ
    timestamp_end = core::get_timestamp() + inc;
    _tx->set_value(VHIGH);
    while (time_after(timestamp_end, core::get_timestamp()));
}

s8 sbus::read_byte(void)
{
    u8 c = 0;
    u8 tries = 3;
    timestamp_t timestamp_end = 0;
    timestamp_t inc = core::convert_us_to_timestamp(DEFAULT_BAUD_DELAY_US) - 12*26;

    while (_rx->get_value());      //�ȴ���ʼλ
    timestamp_end = core::get_timestamp() + inc*1.5;
    while (time_after(timestamp_end, core::get_timestamp()));

    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<0); } //��0λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<1); } //��1λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<2); } //��2λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<3); } //��3λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<4); } //��4λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<5); } //��5λ
    while (time_after(timestamp_end, core::get_timestamp()));
     timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<6); } //��6λ
    while (time_after(timestamp_end, core::get_timestamp()));
    timestamp_end = core::get_timestamp() + inc;
    if (_rx->get_value()) { c |= (0x01<<7); } //��7λ
    while (time_after(timestamp_end, core::get_timestamp()));

    //��ָ����ʱ������Ѱ����λ��
    while (--tries != 0) {
        timestamp_end = core::get_timestamp() + inc/3;
        if (_rx->get_value()) { break;  }       //�յ�����λ���˳�
        while (time_after(timestamp_end, core::get_timestamp()));
    }

     return c;
}

}

/***********************************************************************
** End of file
***********************************************************************/
