/*
------------------------------------------------------------------------------
~ File   : i2c_memory.h
~ Author : Majid Derhambakhsh
~ Version: V0.0.1
~ Created: 07/10/2019 04:38:00 AM
~ Brief  :
~ Support: 
           E-Mail : Majid.Derhambakhsh@gmail.com (subject: Embedded Library Support)
		   
           Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:    Support Serial EEPROM Products With the I2C peripheral.
				  Support AVR - ARM (STM32) Microcontroller Series.

~ Attention  :    This module required [stm32_i2c - I2C_UNIT] Driver Module. 

~ Changes    :
				  Fix : _MEM_SIZE_ERROR error fix
------------------------------------------------------------------------------
*/

#ifndef __I2C_MEMORY_H_
#define __I2C_MEMORY_H_

/* ------------------------------------------------ Includes ------------------------------------------------- */

#include <stdint.h> /* Import standard integer lib */
#include "i2c_memory_conf.h" /* Import config file */

/* ------------------------------------------------------------------ */

#ifdef __CODEVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

#include "I2C_UNIT/i2c_unit.h" /* Import i2c lib */
#include "GPIO_UNIT/gpio_unit.h" /* Import gpio lib */

#include <delay.h>       /* Import delay library */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__)  /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

#include "I2C_UNIT/i2c_unit.h" /* Import i2c lib */
#include "GPIO_UNIT/gpio_unit.h" /* Import gpio lib */

#include <util/delay.h>  /* Import delay library */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

	/* ------------------------------------------------------- */

	#if defined ( __ICCARM__ ) /* ICCARM Compiler */

		#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */

	#elif defined   (  __GNUC__  ) /* GNU Compiler */

		#pragma diag_suppress 177     /* Disable 'unused function' warning */

	#endif /* __ICCARM__ */

	/* ------------------------------------------------------- */

#include "STM32_I2C/stm32_i2c.h" /* Import i2c lib */

/* ------------------------------------------------------------------ */

#else                     /* Compiler not found */

#error Chip or I2C Library not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/* ------------------------------------------------------------------ */

/* ------------------------------------------------- Defines ------------------------------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~ Memory IC Address ~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define _A0PIN_BIT 1 /* Address pin bit in device address */
#define _A1PIN_BIT 2 /* Address pin bit in device address */
#define _A2PIN_BIT 3 /* Address pin bit in device address */

/* ~~~~~~ If User deleted pin state ~~~~~~ */

#ifndef _MEMORY_A0_PIN_STATE
	#define _MEMORY_A0_PIN_STATE 0 /* Address pin in device address */
#endif /* _EEPROM_A0_PIN_STATE */

#ifndef _MEMORY_A1_PIN_STATE
	#define _MEMORY_A1_PIN_STATE 0 /* Address pin in device address */
#endif /* _EEPROM_A1_PIN_STATE */

#ifndef _MEMORY_A2_PIN_STATE
	#define _MEMORY_A2_PIN_STATE 0 /* Address pin in device address */
#endif /* _EEPROM_A2_PIN_STATE */

/* ~~~~~~ Check value of A0 ~ A2 pin ~~~~~~ */

#if ( (_MEMORY_A0_PIN_STATE != 0) && (_MEMORY_A0_PIN_STATE != 1) )
	#warning [MEM WARN01] --> "Your A0 Pin state in not correct"
#endif /* Pin State */

#if ( (_MEMORY_A1_PIN_STATE != 0) && (_MEMORY_A1_PIN_STATE != 1) )
	#warning [MEM WARN01] --> "Your A1 Pin state in not correct"
#endif /* Pin State */

#if ( (_MEMORY_A2_PIN_STATE != 0) && (_MEMORY_A2_PIN_STATE != 1) )
	#warning [MEM WARN01] --> "Your A2 Pin state in not correct"
#endif /* Pin State */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~ Memory IC Parameter ~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifdef _AT24C01

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_8BIT  /* Memory address size */
	#define _MEM_PAGE_SIZE    8U                    /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  16U                   /* Number of memory page */
	#define _MEM_STWC         10U                   /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C02)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_8BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    8U                   /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  32U                  /* Number of memory page */
	#define _MEM_STWC         10U                  /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C04)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_8BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    16U                  /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  32U                  /* Number of memory page */
	#define _MEM_STWC         10U                  /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C08)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_8BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    16U                  /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  64U                  /* Number of memory page */
	#define _MEM_STWC         10U                  /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C16)

	#define _DEV_ADDRESS      0xA0 /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_8BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    16U                  /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  128U                 /* Number of memory page */
	#define _MEM_STWC         10U                  /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C32)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    32U                   /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  128U                  /* Number of memory page */
	#define _MEM_STWC         10U                   /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C64)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    32U                   /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  256U                  /* Number of memory page */
	#define _MEM_STWC         10U                   /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C128)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    64U                   /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  256U                  /* Number of memory page */
	#define _MEM_STWC         5U                    /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C256)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    64U                   /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  512U                  /* Number of memory page */
	#define _MEM_STWC         5U                    /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C512)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT) | (_MEMORY_A0_PIN_STATE << _A0PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    128UL                  /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  512UL                  /* Number of memory page */
	#define _MEM_STWC         5U                    /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#elif defined(_AT24C1024)

	#define _DEV_ADDRESS      (0xA0 | (_MEMORY_A2_PIN_STATE << _A2PIN_BIT) | (_MEMORY_A1_PIN_STATE << _A1PIN_BIT)) /* Memory IC Address */
	
	#define _MEMADD_SIZE      I2C_MEMADD_SIZE_16BIT /* Memory address size */
	#define _MEM_PAGE_SIZE    256UL                  /* Size of memory page */
	#define _MEM_NUM_OF_PAGE  512UL                  /* Number of memory page */
	#define _MEM_STWC         5U                    /* Self Timed Write Cycle */
	
	#define _MEM_MAX_SIZE     ((_MEM_PAGE_SIZE) * (_MEM_NUM_OF_PAGE)) /* Maximum memory size */

#else
	#error [MEM ERROR01]Memory is not selected Or not supported.
#endif /* _24C01 */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#define _TRIALS  10 /* Number of test */

#ifndef _MEM_SIZE_ERROR
	#define _MEM_SIZE_ERROR 2 /* Error value */
#endif /* _MEM_SIZE_ERROR */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ By compiler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifdef __CODEVISIONAVR__  /* Check compiler */

#define _ERROR_VAL             _STAT_ERROR             /* OK status value */
#define _OK_VAL                _STAT_OK                /* OK status value */
#define I2C_MEMADD_SIZE_8BIT   _I2C_MEMADD_SIZE_8BIT   /* Memory Address Size */
#define I2C_MEMADD_SIZE_16BIT  _I2C_MEMADD_SIZE_16BIT  /* Memory Address Size */

#define _I2C_MEM_READY(da,tr,tim)            I2C_IsDeviceReady((da),(tr),(tim)) /* Change function */
#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_ERASE(da,ma,mas,qu,tim)     I2C_Mem_Erase((da),(ma),(mas),(qu),(tim)) /* Change function */
#define _MEM_WP_CONTROL(ps)                  GPIO_WritePin(&_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     delay_ms((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#elif defined(__GNUC__)  /* Check compiler */

#define _ERROR_VAL             _STAT_ERROR             /* OK status value */
#define _OK_VAL                _STAT_OK                /* OK status value */
#define I2C_MEMADD_SIZE_8BIT   _I2C_MEMADD_SIZE_8BIT   /* Memory Address Size */
#define I2C_MEMADD_SIZE_16BIT  _I2C_MEMADD_SIZE_16BIT  /* Memory Address Size */

#define _I2C_MEM_READY(da,tr,tim)            I2C_IsDeviceReady((da),(tr),(tim)) /* Change function */
#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  I2C_Mem_Write((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   I2C_Mem_Read((da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_ERASE(da,ma,mas,qu,tim)     I2C_Mem_Erase((da),(ma),(mas),(qu),(tim)) /* Change function */
#define _MEM_WP_CONTROL(ps)                  GPIO_WritePin(&_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     _delay_ms((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#elif defined(USE_HAL_DRIVER)  /* Check driver */

#define _ERROR_VAL    HAL_ERROR             /* OK status value */
#define _OK_VAL       HAL_OK                /* OK status value */
#define _GPIO_PIN_RESET                     GPIO_PIN_RESET /* Select GPIO reset instruction */
#define _GPIO_PIN_SET                       GPIO_PIN_SET /* Select GPIO set instruction */

#define _I2C_MEM_READY(da,tr,tim)            HAL_I2C_IsDeviceReady(&_CONNECTED_I2C,(da),(tr),(tim)) /* Change function */
#define _I2C_MEM_WRITE(da,ma,mas,md,qu,tim)  HAL_I2C_Mem_Write2(&_CONNECTED_I2C,(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_READ(da,ma,mas,md,qu,tim)   HAL_I2C_Mem_Read2(&_CONNECTED_I2C,(da),(ma),(mas),(md),(qu),(tim)) /* Change function */
#define _I2C_MEM_ERASE(da,ma,mas,qu,tim)     HAL_I2C_Mem_Erase(&_CONNECTED_I2C,(da),(ma),(mas),(qu),_MEM_STWC,(tim)) /* Change function */
#define _MEM_WP_CONTROL(ps)                  HAL_GPIO_WritePin(_WRITE_PROTECT_PORT,(1 << _WRITE_PROTECT_PIN),(ps)) /* Change function */

#ifndef _DELAY_MS
	#define _DELAY_MS(t)                     HAL_Delay((t)) /* Change function */
#endif /* _DELAY_MS */

/* ------------------------------------------------------------------ */

#else 
#endif /* __CODEVISIONAVR__ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* -------------------------------------------------- Enum --------------------------------------------------- */

/* ------------------------------------------------- Struct -------------------------------------------------- */

/* ............ By HAL DRIVER ............ */

#ifdef USE_HAL_DRIVER

extern I2C_HandleTypeDef _CONNECTED_I2C;

#endif /* USE_HAL_DRIVER */

/* ------------------------------------------------ Prototype ------------------------------------------------ */

void I2C_Memory_Init(void); /* This function is for initialize memory */
/*
	Guide   :
			 Parameters    :
			 
			 Return Values :
			
	Example :
			 I2C_Memory_Init();
	
*/

uint8_t I2C_MemoryIsReady(uint16_t time_out); /* Checks if memory is ready for communication */
/*
	Guide   :
			 Parameters    :
							time_out : Timeout duration.
			 Return Values :
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t device_status;
			 
			 device_status = I2C_MemoryIsReady(50);
	
*/

uint8_t I2C_Memory_SingleWrite(uint32_t address,uint8_t udata,uint16_t time_out); /* This function is for write data to memory */
/*
	Guide   :
			 Parameters    :
							address   : Internal memory address.
							udata     : Data for write to memory.
							time_out  : Timeout duration.
			 Return Values :
							_MEM_SIZE_ERROR
							
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t com_status;
			 
			 com_status = I2C_Memory_SingleWrite(0,'A',50);
	
*/

uint8_t I2C_Memory_BurstWrite(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out); /* This function is for write data to memory */
/*
	Guide   :
			 Parameters    :
							address   : Internal memory address.
							udata     : Pointer to data buffer.
							size      : Amount of data to be send.
							time_out  : Timeout duration.
			 Return Values :
							_MEM_SIZE_ERROR
							
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t com_status;
			 
			 com_status = I2C_Memory_BurstWrite(0,"Hello",5,50);
	
*/

uint8_t I2C_Memory_SingleRead(uint32_t address,uint8_t *udata,uint16_t time_out); /* This function is for read data from memory */
/*
	Guide   :
			 Parameters    :
							address   : Internal memory address.
							udata     : Variable for read data from memory.
							time_out  : Timeout duration.
			 Return Values :
							_MEM_SIZE_ERROR
							
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t my_data;
			 uint8_t com_status;
			 
			 com_status = I2C_Memory_SingleRead(0,&my_data,50);
	
*/

uint8_t I2C_Memory_BurstRead(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out); /* This function is for read data from memory */
/*
	Guide   :
			 Parameters    :
							address   : Internal memory address.
							udata     : Pointer to data buffer.
							size      : Amount of data to be read.
							time_out  : Timeout duration.
			 Return Values :
							_MEM_SIZE_ERROR
			
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t com_status;
			 uint8_t received_data[10];
			 
			 com_status = I2C_Memory_BurstRead(12,received_data,10,50);
	
*/

uint8_t I2C_Memory_Erase(uint32_t address,uint32_t quantity,uint16_t time_out); /* This function is for erase data of memory */
/*
	Guide   :
			 Parameters    :
							address   : Internal memory address.
							quantity  : Amount of data to be erase.
							time_out  : Timeout duration.
			 Return Values :
							_MEM_SIZE_ERROR
							
							AVR : _STAT_OK / _STAT_ERROR
							ARM : HAL_OK / HAL_ERROR
			
	Example :
			 uint8_t com_status;
			 
			 com_status = I2C_Memory_Erase(0,65000,50);
	
*/

#endif /* __I2C_EEPROM_H_ */
