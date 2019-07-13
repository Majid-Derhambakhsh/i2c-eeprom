/*
------------------------------------------------------------------------------
~ File   : i2c_unit.h
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

#ifndef __I2C_UNIT_H_
#define __I2C_UNIT_H_

/*************************************** Include ***************************************/

#include <stdint.h> /* Import standard integer type */

#include "i2c_unit_conf.h" /* Import i2c config file */

/*----------------------------------------------------------*/

#ifdef __CODEVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

#include <io.h>            /* Import AVR IO library */
#include <delay.h>         /* Import delay library */

/*----------------------------------------------------------*/

#elif defined(__GNUC__)   /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

#include <avr/io.h>        /* Import AVR IO library */
#include <util/delay.h>    /* Import delay library */

/*----------------------------------------------------------*/

#else                     /* Compiler not found */

#error Compiler not supported  /* Send error */

#endif /* __CODEVISIONAVR__ */

/*************************************** Defines ***************************************/

/* ------------------------------ Type size ------------------------------ */

#ifndef ENUM_U8_T
	#define ENUM_U8_T(ENUM_NAME)   Enum_##ENUM_NAME; typedef uint8_t ENUM_NAME /* Config enum size */
#endif

/* ----------------------Delay - MCU Clock definition -------------------- */

#ifdef __CODEVISIONAVR__ /* Check compiler */

	#define _F_CPU _MCU_CLOCK_FREQUENCY_ /* Define CPU clock */
	
	#ifndef _DELAY_MS
		
		#define _DELAY_MS(x)    delay_ms(x)
		
	#endif /* _DELAY_MS */
	
#elif defined(__GNUC__) /* Check compiler */

	#define _F_CPU F_CPU /* Define CPU clock */
	
	#ifndef _DELAY_MS 
		
		#define _DELAY_MS(x)    _delay_ms(x)
		
	#endif /* _DELAY_MS */

#endif /* __CODEVISIONAVR__ */

/* ------------------------------ Prescaler ------------------------------ */

#define _PRE1   1  /* Prescaler = 1  */
#define _PRE4   4  /* Prescaler = 4  */
#define _PRE16  16 /* Prescaler = 16 */
#define _PRE64  64 /* Prescaler = 64 */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#if _PRESCALER == _PRE1 /* Check prescaler */

	#define _PRE_CODE 0x00 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == _PRE4 /* Check prescaler */

	#define _PRE_CODE 0x01 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == _PRE16 /* Check prescaler */

	#define _PRE_CODE 0x02 /* Prescaler value for set register */

/* ------ */
#elif _PRESCALER == _PRE64 /* Check prescaler */

	#define _PRE_CODE 0x03 /* Prescaler value for set register */

#endif

/* -------------------------------- Memory ------------------------------- */

#define _MEMORY_DEF_VAL                  0xFF /* Default value of memory */
#define _I2C_MEMADD_SIZE_8BIT            8    /* 8Bit memory address */
#define _I2C_MEMADD_SIZE_16BIT           16   /* 16Bit memory address */
#define _P0_SHIFT_VAL_MEMADD_SIZE_8BIT   7    /* Value for memory address shift */
#define _P0_BIT_SEL_MEMADD_SIZE_8BIT     0x0E /* Value for check P0 addressing bit in 8bit memory address mode */
#define _P0_SHIFT_VAL_MEMADD_SIZE_16BIT  15   /* Value for memory address shift */
#define _P0_BIT_SEL_MEMADD_SIZE_16BIT    0x02 /* Value for check P0 addressing bit in 16bit memory address mode */

/* --------------------------------- I2C --------------------------------- */

#define _I2C_DIS                     0    /* Disable value for reset register */
#define _I2C_STATUS                  0xF8 /* Status flag bits */
#define _DEVICE_READ                 0x01 /* Value for set device address */

/* Define Registers */

#define _I2C_SR TWSR /* I2C Status Register */
#define _I2C_DR TWDR /* I2C Data Register */
#define _I2C_CR TWCR /* I2C Control Register */
#define _I2C_AR TWAR /* I2C Address Register */
#define _I2C_BR TWBR /* I2C BaudRate Register */

/* -------------------------------- Public ------------------------------- */

#define _BYTE_SHIFT_VAL              8    /* Value for shift */
#define _MIN_STEP_FOR_DT             3    /* Minimum steps of transmit data to device */
#define _MIN_STEP_FOR_DR             6    /* Minimum steps of receive data from device */
#define _MEMORY_BURST_WRITE_STEPS    4    /* Number of burst write step */
#define _MEMORY_BURST_READ_STEPS     6    /* Number of burst write step */

#ifndef _FALSE
	#define _FALSE 0 /* False value */
#endif

#ifndef _TRUE
	#define _TRUE  1  /* True value */
#endif

/**************************************** Enums ****************************************/

typedef enum /* Enum Status Codes for Master Transmitter Mode */
{
	
	_MT_START_TRANSMITTED				= 0x08,
	_MT_REP_START_TRANSMITTED			= 0x10,
	_MT_SLA_W_TRANSMITTED_ACK			= 0x18,
	_MT_SLA_W_TRANSMITTED_NACK			= 0x20,
	_MT_DATA_TRANSMITTED_ACK			= 0x28,
	_MT_DATA_TRANSMITTED_NACK			= 0x30,
	_MT_SLA_W_ARB_LOST					= 0x38
	
}ENUM_U8_T(I2C_Status_MT_TypeDef);

typedef enum /* Enum Status codes for Master Receiver Mode */
{
	
	_MR_START_TRANSMITTED				= 0x08,
	_MR_REP_START_TRANSMITTED			= 0x10,
	_MR_SLA_R_ARB_LOST					= 0x38,
	_MR_SLA_R_TRANSMITTED_ACK			= 0x40,
	_MR_SLA_R_TRANSMITTED_NACK			= 0x48,
	_MR_DATA_RECEIVED_ACK				= 0x50,
	_MR_DATA_RECEIVED_NACK				= 0x58
	
}ENUM_U8_T(I2C_Status_MR_TypeDef);

typedef enum /* Enum Status Codes for Slave Receiver Mode */
{
	
	_SR_SLA_W_RECEIVED_ACK				= 0x60,
	_SR_SLA_RW_ARB_LOST					= 0x68,
	_SR_GEN_ADDR_RECEIVED_ACK			= 0x70,
	_SSR_SLA_RW_ARB_LOST_2				= 0x78,
	_SR_DATA_RECEIVED_SLA_ACK			= 0x80,
	_SR_DATA_RECEIVED_SLA_NACK			= 0x88,
	_SR_DATA_RECEIVED_GEN_ADDR_ACK		= 0x90,
	_SR_DATA_RECEIVED_GEN_ADDR_NACK		= 0x98,
	_SR_STOP_REP_START					= 0xA0
	
}ENUM_U8_T(I2C_Status_SR_TypeDef);

typedef enum /* Enum Status Codes for Slave Transmitter Mode */
{
	
	_ST_SLA_R_RECEIVED_ACK				= 0xA8,
	_ST_SLA_RW_ARB_LOST					= 0xB0,
	_ST_DATA_TRANSMITTED_ACK			= 0xB8,
	_ST_DATA_TRANSMITTED_NACK			= 0xC0,
	_ST_LAST_DATA_TRANSMITTED_ACK		= 0xC8
	
}ENUM_U8_T(I2C_Status_ST_TypeDef);

typedef enum /* Enum Status Codes */
{
	
	_STAT_ERROR = 0, /* Error status */
	_STAT_OK    = 1  /* OK status */
	
}ENUM_U8_T(StatusTypeDef);

/************************************** Prototype **************************************/

uint8_t I2C_Status(void); /* Function for take I2C status */
/*
	Parameters    :
	
	Return Values :
					I2C Status.
	
	Example       :
					uint8_t twi_status;
			
					twi_status = I2C_Status();
			
*/

uint8_t I2C_BeginTransmission(uint16_t time_out); /* Function for send START condition */
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					I2C Status.
	
	Example       :
					uint8_t twi_status;
			
					twi_status = I2C_BeginTransmission(100);
			
*/

uint8_t I2C_Transmit(uint8_t data , uint16_t time_out); /* Function for transmit data */
/*
	Parameters    :
					data : data for send to i2c.
					time_out : Timeout duration.
	
	Return Values :
					I2C Status.
	
	Example       :
					uint8_t twi_status;
			
					twi_status = I2C_Transmit(0xA0,100); (Send i2c device address)
			
*/

uint8_t I2C_ReceiveACK(uint16_t time_out); /* Function for receive data with ACK */
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					Received data.
	
	Example       :
					uint8_t my_data;
			
					my_data = I2C_ReceiveACK(100);
			
*/

uint8_t I2C_ReceiveNACK(uint16_t time_out); /* Function for receive data with no ACK */
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					Received data.
	
	Example       :
					uint8_t my_data;
			
					my_data = I2C_ReceiveNACK(100);
			
*/

void I2C_EndTransmission(uint16_t time_out); /* Function for stop transmission I2C */
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
	
	Example       :
					I2C_EndTransmission(250);
			
*/

void I2C_SetAddress(uint8_t address); /* Function for self I2C address */
/*
	Parameters    :
					address : Slave address of the TWI unit.
	
	Return Values :
	
	Example       :
					I2C_SetAddress(0x50);
			
*/

void I2C_Init(void); /* This Function is for Initialize the I2C peripheral. */
/*
	Parameters    :
	
	Return Values :
	
	Example       :
					I2C_Init();
			
*/

void I2C_DeInit(void); /* This Function is for DeInitialize the I2C peripheral. */
/*
	Parameters    :
	
	Return Values :
	
	Example       :
					I2C_DeInit();
			
*/

uint8_t I2C_IsDeviceReady(uint8_t dev_address , uint16_t trials , uint16_t time_out); /* Checks if target device is ready for communication. */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					trials : Number of trials.
					time_out : Timeout duration.
	
	Return Values :
					_TRUE / _FALSE
	
	Example       :
					uint8_t com_stat;
					
					com_stat = I2C_IsDeviceReady(0xD2,5,50); (0xD2 : Device Address)
			
*/

StatusTypeDef I2C_Master_Transmit(uint8_t dev_address , uint8_t *data , uint32_t quantity , uint16_t time_out); /* This function is for send data to i2c device */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					data : Pointer to data buffer.
					quantity : Amount of data to be sent.
					time_out : Timeout duration.
	
	Return Values :
					_STAT_OK / _STAT_ERROR
	
	Example       :
					uint8_t com_stat;
					
					ucom_stat = I2C_Master_Transmit(0xD2,"Hello",5,100); (0xD2 : Device Address)
			
*/

StatusTypeDef I2C_Master_Receive(uint8_t dev_address , uint8_t *data , uint32_t quantity , uint16_t time_out); /* This function is for receive data from i2c device */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					data : Pointer to data buffer.
					quantity : Amount of data to be receive.
					time_out : Timeout duration.
	
	Return Values :
					_STAT_OK / _STAT_ERROR
	
	Example       :
					uint8_t com_stat;
					uint8_t my_received_data[10];
					
					com_stat = I2C_Master_Receive(0xD2,my_received_data,5,100); (0xD2 : Device Address)
			
*/

StatusTypeDef I2C_Mem_Write(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint8_t *mem_data , uint32_t quantity , uint16_t time_out); /* This function is for write data to external memory */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					mem_address : Internal memory address.
					
					mem_add_size : Size of internal memory address. this
								   parameter is :
												 _I2C_MEMADD_SIZE_8BIT
												 _I2C_MEMADD_SIZE_16BIT
					
					mem_data : Pointer to data buffer.
					quantity : Amount of data to be sent.
					time_out : Timeout duration.
	
	Return Values :
					_STAT_OK / _STAT_ERROR
	
	Example       :
					uint8_t com_stat;
					uint8_t my_data[21] = "I AM Microcontroller";
					
					com_stat = I2C_Mem_Write(0xA0 , 75 , _I2C_MEMADD_SIZE_16BIT , my_data , 20 , 100); (0xA0 : Device Address)
			
*/

StatusTypeDef I2C_Mem_Read(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint8_t *mem_data , uint32_t quantity , uint16_t time_out ); /* This function is for read data from external memory */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					mem_address : Internal memory address.
					
					mem_add_size : Size of internal memory address. this
								   parameter is :
												 _I2C_MEMADD_SIZE_8BIT
												 _I2C_MEMADD_SIZE_16BIT
					
					mem_data : Pointer to data buffer.
					quantity : Amount of data to be receive.
					time_out : Timeout duration.
	
	Return Values :
					_STAT_OK / _STAT_ERROR
	
	Example       :
					uint8_t com_stat;
					uint8_t my_data[20];
					
					com_stat = I2C_Mem_Read(0xA0 , 0 , _I2C_MEMADD_SIZE_16BIT , my_data , 10 , 100); (0xA0 : Device Address)
			
*/

StatusTypeDef I2C_Mem_Erase(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint32_t quantity , uint16_t time_out); /* This function is for erase external memory */
/*
	Parameters    :
					dev_address : Target device address: The device 7 bits 
								  address value in datasheet must be shift
								  at right before call interface.

					mem_address : Internal memory address.
					
					mem_add_size : Size of internal memory address. this
								   parameter is :
												 _I2C_MEMADD_SIZE_8BIT
												 _I2C_MEMADD_SIZE_16BIT
					
					quantity : Amount of data to be erase.
					time_out : Timeout duration.
	
	Return Values :
					_STAT_OK / _STAT_ERROR
	
	Example       :
					uint8_t com_stat;
					
					com_stat = I2C_Mem_Read(0xA0 , 0 , _I2C_MEMADD_SIZE_16BIT , 10 , 100); (0xA0 : Device Address)
			
*/

#endif /* __I2C_UNIT_H_ */
