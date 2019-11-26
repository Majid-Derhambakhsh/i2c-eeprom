/*
------------------------------------------------------------------------------
~ File   : stm32_i2c_conf.h
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

#ifndef __STM32_I2C_CONF_H_
#define __STM32_I2C_CONF_H_

/* -------- Configuration -------- */

#define STM32F1

#define _CONNECTED_I2C hi2c1

#define _MEM_DEF_VAL_BUFF_LENGTH 50

/*
	Guide   :
			  #define STM32Xx                    : STM32 Family.
			  #define _CONNECTED_I2C hi2c        : I2C structure to use in functions.
			  
			  #define _MEM_DEF_VAL_BUFF_LENGTH x : Buffer length for erase memory.
												   Increase length == Increase speed.
			  
	Example :
			  #define STM32H7
			  
			  #define _CONNECTED_I2C hi2c1
			  
			  #define _MEM_DEF_VAL_BUFF_LENGTH 100
*/

#endif /* __I2C_EEPROM_CONF_H_ */
