#ifndef __PRINTK_H__
#define __PRINTK_H__


#define __DEBUG

////=============================================
#if 1
#ifdef __DEBUG
  extern int _printk(const char *format, ...);
  #define printk	_printk
#else
  extern int _printk_dummy(const char *format, ...);
  #define printk	_printk_dummy1
#endif
#endif
  
#if 0  
#include <stdio.h>
  
#define _printk	printf
#define printk	printf
#endif
////=============================================
#endif ////__PRINTK_H__



	
