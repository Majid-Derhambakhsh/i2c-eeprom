/*
------------------------------------------------------------------------------
~ File   : i2c_memory_conf.c
~ Author : Majid Derhambakhsh
~ Version: V0.0.0
~ Created: 07/10/2019 04:38:00 AM
~ Brief  :
~ Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
~ Description:    Support Serial EEPROM Products With the I2C peripheral.
				  Support AVR - ARM (STM32) Microcontroller Series.

~ Attention  :    This module required [stm32_i2c - I2C_UNIT] Driver Module. 
------------------------------------------------------------------------------
*/

#ifndef __I2C_MEMORY_CONF_H_
#define __I2C_MEMORY_CONF_H_

/* ----------- Configuration ----------- */

#define _AT24C04

#define _MEMORY_A0_PIN_STATE 0
#define _MEMORY_A1_PIN_STATE 0
#define _MEMORY_A2_PIN_STATE 0

/* ---- Write Protect Configuration ---- */

#define _WP_DISABLE

#define _WRITE_PROTECT_PORT  PORTB
#define _WRITE_PROTECT_PIN   0

/*
	Guide   :
			  #define _AT24Cxx               : Your memory code.
			  #define _MEMORY_Ax_PIN_STATE Y : Ax pins state.
			  
				Ax value is between 0 ~ 2.
				Y is 0 Or 1.
				
			  #define _WP_DISABLE            : Disable Write Protect. For enable it comment this define.
			  
			  #define _WRITE_PROTECT_PORT  x : Write Protect Port for Hardware and Software Data Protection.
				
				AVR : PORTx  :  where x can be (A..H) to select the GPIO peripheral.
				ARM : GPIOx  :  where x can be (A..H) to select the GPIO peripheral for STM32 family 
				
			  #define _WRITE_PROTECT_PIN   x : Write Protect Pin for Hardware and Software Data Protection.
											   This parameter can (0..15). 

Example :
			  #define _AT24C04

			  #define _MEMORY_A0_PIN_STATE 0
			  #define _MEMORY_A1_PIN_STATE 0
			  #define _MEMORY_A2_PIN_STATE 0
			  
			  //#define _WP_DISABLE

			  #define _WRITE_PROTECT_PORT  PORTB
			  #define _WRITE_PROTECT_PIN   5
*/

#endif /* __I2C_EEPROM_CONF_H_ */
