/*
------------------------------------------------------------------------------
~ File   : i2c_unit_conf.c
~ Author : Majid Derhambakhsh
~ Version: V0.7.0
~ Created: 07/04/2019 08:58:00 AM
~ Brief  :
~ Support: Majid.do16@gmail.com
------------------------------------------------------------------------------
~ Description:

~ Attention  :    This file is for AVR microcontroller
				  

~ Changes    :
				  Add : I2C_DeInit            function for DeInitialize the I2C peripheral.
				  Add : I2C_IsDeviceReady     function for Checks if target device is ready for communication.
				  Add : I2C_Master_Transmit   function for send data to i2c devices.
				  Add : I2C_Master_Receive    function for receive data from i2c devices.
				  Add : I2C_Mem_Write         function for write data to external i2c memory.
				  Add : I2C_Mem_Read          function for read data from external i2c memory.
				  Add : I2C_Mem_Erase         function for erase external i2c memory.
				  
				  Add : time_out for exit from instruction.
------------------------------------------------------------------------------
*/

#ifndef __I2C_UNIT_CONF_H_
#define __I2C_UNIT_CONF_H_

/* -------------------- Define -------------------- */

#define _F_SCL      100000UL 
#define _PRESCALER  _PRE1 

/*
	Guide :
			_F_SCL	   : Specifies the clock frequency.
						 This parameter must be set to a value lower than 400kHz
			
			_PRESCALER : i2c prescaler value argument is
						 _PRE1 , _PRE4 , _PRE16 , _PRE64
*/

/* ------------------------------------------------ */

#endif /* __I2C_UNIT_CONFIG_H_ */
