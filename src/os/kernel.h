/*******************************Copyright (c)***************************
** 
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

namespace os {

/**
 *  @enum  os_error_code
 *  @brief ����ϵͳ-������
 */
enum os_error_code
{
	ERR_NO = 0,
	ERR_UNKNOWN,
	ERR_INVALID_HANDLE_R,
	ERR_INVALID_PARAM,
	ERR_OUT_MEMORY,
	ERR_OPERATION_TIMEOUT,
	ERR_MSGQUE_FULL,
	ERR_OPERATION_FAILED,
	ERR_OPERATION_UNSUPPORTED,
	ERR_APPLICATION,
};

enum wait_mode
{
	WAIT_FOREVER = ~(0),
	DO_NOT_WAIT = 0,
};


class os_object
{
protected:
	HANDLE _handle;
	PCSTR  _name;

public:
	os_object(void) : _handle(NULL), _name("DEF_OS_OBJECT")
	{

	}

	HANDLE get_handle(void) const	{ return _handle; }
	PCSTR get_name(void) const		{ return _name; }
};

class kernel
{
public:
	kernel(void);
	~kernel(void);

public:
	static void systick_init(void);
	static void init(void);
	static void start(void);
	static void exit(void);
	static u32 get_cpu_load(void);
	static u32 get_tick_period(void);
	static u32 convertmstotick(s32 ms);
	static void on_error(enum os_error_code errcode, os_object* pobject);
};

}
/***********************************************************************
** End of file
***********************************************************************/




