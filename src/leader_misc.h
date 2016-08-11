/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "leader_type.h"
#include "leader.h"

/* ��ӡ��ʱ�����Ϣ*/
#define ERR(fmt, ...) print(3, ("[ERR] --%s--(%d)-<%s>: " fmt), \
    __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#define WRN(fmt, ...) print(2, ("[WRN]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define INF(fmt, ...) print(1, ("[INF]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)
#define DBG(fmt, ...) print(0, ("[DBG]<%s>: " fmt), __FUNCTION__, ##__VA_ARGS__)

#define MIN(a, b)                           ((a) > (b) ? (b) : (a))
#define MAX(a, b)                           ((a) < (b) ? (b) : (a))
#define CEIL_DIV(a, b)                      (((a) + (b) - 1) / (b))	// ��������£
#define CEIL_ALIGN(a, b)                    (CEIL_DIV((a), (b)) * (b))
#define ARRAYSIZE(arr)                      (sizeof(arr) / sizeof((arr)[0]))
#define OFFSETOF(_type, _member)	        ((u32) (&((_type *)0)->_member))
#define container_of(_ptr, _type, _member)	((_type *)((char *)_ptr - OFFSETOF(_type, _member)));
#define UNREFERENCED_PARAMETER(p)           { (p) = (p); }

// TODO:ʧ�ر���:��Ը��ӵ���绷���ĸ��ţ�������δ֪���쳣(����hardfault)��������������ʱ�Լ�δ֪�Ĵ����һ������MCU
// �����쳣�Ĵ���dump��ARM core�Ĵ�����pc sp��ֵ���Ա�ͨ��map�ļ���λ�쳣��תǰ����һ��ָ��,
// ����ִ�е�����Ϊ0��ָ����߿�ָ�룬����Ӳ���쳣�ȣ�����ͨ�����stm32f4xx_it.c�е��쳣���������dump��ջ��Ϣ,
// ���磺TI sysbios�����׳��쳣ʱ��ӡ��CPU�˼Ĵ���pc,sp�ȣ�������map�ļ�������ҵ�����ʱ�Ķ�ջ;
// ���磺Linux��oops������ִ�е���ָ���ʱ���׳���ջ��Ϣ,
// map�ڴ�ֲ��ļ��������˳�����ص��ڴ�ʱ�ķֲ����������������˶�ջ�Լ����������������ĵ�ַ�������Ҫ��Ϣ
//#define ASSERT(_expr)			((_expr) ? NULL : ERR("Assert failure.\n"))
#define ASSERT(_expr)			if (!(_expr)) {ERR("Assert failure.\n");} 
	// ERR("ASSERT.\n")
	// reset    // ��������ֱ������
	// while(1) // �����ڴ˴�����������鿴�����,�������ڵ��ԣ�����ʹ�� */

#define CAPTURE_ERR()						while(1)// ͨ�ô��󲶻�


#define BYTE_SIZE							(1)
#define HALF_WORD_SIZE						(2)
#define WORD_SIZE							(4)

#if defined(WIN32)
	#define	TARGET_WINDOWS
#else
	#define	TARGET_STM32
#endif

#if !defined(__cplusplus)
	#define inline							__inline
#endif

#define USE_UCOS3                           0
#define USE_STM32F4_DEMO                    0

/* Useful constants.  */
#define M_E_F			2.7182818284590452354f
#define M_LOG2E_F		1.4426950408889634074f
#define M_LOG10E_F		0.43429448190325182765f
#define M_LN2_F			_M_LN2_F
#define M_LN10_F		2.30258509299404568402f
#define M_PI_F			3.14159265358979323846f
#define M_TWOPI_F       (M_PI_F * 2.0f)
#define M_PI_2_F		1.57079632679489661923f
#define M_PI_4_F		0.78539816339744830962f
#define M_3PI_4_F		2.3561944901923448370E0f
#define M_SQRTPI_F      1.77245385090551602792981f
#define M_1_PI_F		0.31830988618379067154f
#define M_2_PI_F		0.63661977236758134308f
#define M_2_SQRTPI_F	1.12837916709551257390f
#define M_DEG_TO_RAD_F 	0.01745329251994f
#define M_RAD_TO_DEG_F 	57.2957795130823f
#define M_SQRT2_F		1.41421356237309504880f
#define M_SQRT1_2_F		0.70710678118654752440f
#define M_LN2LO_F       1.9082149292705877000E-10f
#define M_LN2HI_F       6.9314718036912381649E-1f
#define M_SQRT3_F		1.73205080756887719000f
#define M_IVLN10_F      0.43429448190325182765f /* 1 / log(10) */
#define M_LOG2_E_F      _M_LN2_F
#define M_INVLN2_F      1.4426950408889633870E0f  /* 1 / log(2) */
#define M_DEG_TO_RAD 	0.01745329251994
#define M_RAD_TO_DEG 	57.2957795130823
//			 ��       ����      �̳�
// public 	 �ɷ���   �ɷ���	 �ɼ̳�
//
// protected �ɷ���   ���ɷ���  �ɼ̳�
//
// private   �ɷ���   ���ɷ���  ���ɼ̳�

/***********************************************************************
** End of file
***********************************************************************/