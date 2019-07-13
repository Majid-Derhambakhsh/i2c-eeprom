/*
------------------------------------------------------------------------------
~ File   : stm32_i2c.h
~ Author : Majid Derhambakhsh
~ Version: V0.0.0
~ Created: 07/09/2019 08:12:00 AM
~ Brief  :
~ Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
~ Description:    Support ARM (STM32) Microcontroller Series.

~ Attention  :    This module required HAL Driver.

~ Changes    :

------------------------------------------------------------------------------
*/

#ifndef __STM32_I2C_H_
#define __STM32_I2C_H_

/*************************************** Include ***************************************/

#include <stdint.h> /* Import standard integer type */
#include "stm32_i2c_conf.h" /* Import config file */

/* ------------------------------------------------------------------ */

#ifdef USE_HAL_DRIVER  /* Check driver */

	/* --------------- Check Mainstream series --------------- */

	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */

	/* ------------ Check High Performance series ------------ */

	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */

	/* ------------ Check Ultra low power series ------------- */

	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	#else
	#endif /* STM32F1 */
	
	/* ------------------------------------------------------- */
	
	#if defined ( __ICCARM__ ) /* ICCARM Compiler */
	
	#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */
	
	#elif defined   (  __GNUC__  ) /* GNU Compiler */
	
	#pragma diag_suppress 177     /* Disable 'unused function' warning */
	
	#endif /* __ICCARM__ */

#endif /* USE_HAL_DRIVER */

/* ------------------------------------------------------------------ */

/*************************************** Defines ***************************************/

/* -------------------------------- Memory ------------------------------- */

#define _MEMORY_DEF_VAL                  0xFF /* Default value of memory */
#define _P0_SHIFT_VAL_MEMADD_SIZE_8BIT   7    /* Value for memory address shift */
#define _P0_BIT_SEL_MEMADD_SIZE_8BIT     0x0E /* Value for check P0 addressing bit in 8bit memory address mode */
#define _P0_SHIFT_VAL_MEMADD_SIZE_16BIT  15   /* Value for memory address shift */
#define _P0_BIT_SEL_MEMADD_SIZE_16BIT    0x02 /* Value for check P0 addressing bit in 16bit memory address mode */

#define _CALC_DEVADD_8BIT(dev_add , mem_add)   ((dev_add) | (uint8_t)(((mem_add) >> _P0_SHIFT_VAL_MEMADD_SIZE_8BIT) & _P0_BIT_SEL_MEMADD_SIZE_8BIT)) /* Calculating new address */
#define _CALC_DEVADD_16BIT(dev_add , mem_add)  ((dev_add) | (uint8_t)(((mem_add) >> _P0_SHIFT_VAL_MEMADD_SIZE_16BIT) & _P0_BIT_SEL_MEMADD_SIZE_16BIT)) /* Calculating new address */

/* -------------------------------- Buffer ------------------------------- */

#if (_MEM_DEF_VAL_BUFF_LENGTH < 5)

	#warning "Your buffer length is low"

#endif /* _MEM_DEF_VAL_BUFF_LENGTH */

/**************************************** Enums ****************************************/

/************************************** Prototype **************************************/

HAL_StatusTypeDef HAL_I2C_Mem_Write2(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout); /* This function is for write data to external memory */
/*
	Parameters    :
					hi2c       : Pointer to a I2C_HandleTypeDef structure that contains
					             the configuration information for the specified I2C.
					
					DevAddress : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					MemAddress : Internal memory address.
					
					MemAddSize : Size of internal memory address. this
								   parameter is :
												 I2C_MEMADD_SIZE_8BIT
												 I2C_MEMADD_SIZE_16BIT
					
					pData	   : Pointer to data buffer.
					Size	   : Amount of data to be sent.
					Timeout    : Timeout duration.
	
	Return Values :
					HAL_OK / HAL_ERROR
	
	Example       :
					uint8_t com_stat;
					uint8_t my_data[21] = "I AM Microcontroller";
					
					com_stat = HAL_I2C_Mem_Write2(&hi2c1 , 0xA0 , 75 , I2C_MEMADD_SIZE_16BIT , my_data , 20 , 100); (0xA0 : Device Address)
			
*/

HAL_StatusTypeDef HAL_I2C_Mem_Read2(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout); /* This function is for read data from external memory */
/*
	Parameters    :
					hi2c       : Pointer to a I2C_HandleTypeDef structure that contains
					             the configuration information for the specified I2C.
					
					DevAddress : Target device address: The device 7 bits 
								 address value in datasheet must be shift
								 at right before call interface.

					MemAddress : Internal memory address.
					
					MemAddSize : Size of internal memory address. this
								   parameter is :
												 I2C_MEMADD_SIZE_8BIT
												 I2C_MEMADD_SIZE_16BIT
					
					pData	   : Pointer to data buffer.
					Size	   : Amount of data to be read.
					Timeout    : Timeout duration.
	
	Return Values :
					HAL_OK / HAL_ERROR
	
	Example       :
					uint8_t com_stat;
					uint8_t my_data[15];
					
					com_stat = HAL_I2C_Mem_Read2(&hi2c1 , 0xA0 , 75 , I2C_MEMADD_SIZE_16BIT , my_data , 10 , 100); (0xA0 : Device Address)
			
*/

HAL_StatusTypeDef HAL_I2C_Mem_Erase(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint16_t Size, uint32_t stwc, uint32_t Timeout); /* This function is for erase external memory */
/*
	Parameters    :
					hi2c       : Pointer to a I2C_HandleTypeDef structure that contains
					             the configuration information for the specified I2C.
					
					DevAddress : Target device address: The device 7 bits 
								 address value in datasheet must be shift
								 at right before call interface.

					MemAddress : Internal memory address.
					
					MemAddSize : Size of internal memory address. this
								   parameter is :
												 I2C_MEMADD_SIZE_8BIT
												 I2C_MEMADD_SIZE_16BIT
					
					Size       : Amount of data to be read.
					stwc       : Self timed write cycle.
					Timeout    : Timeout duration.
	
	Return Values :
					HAL_OK / HAL_ERROR
	
	Example       :
					uint8_t com_stat;
					
					com_stat = HAL_I2C_Mem_Erase(&hi2c1 , 0xA0 , 75 , I2C_MEMADD_SIZE_16BIT , 10 , 5 , 100); (0xA0 : Device Address)
			
*/

#endif /* __stm32_i2c_H_ */
