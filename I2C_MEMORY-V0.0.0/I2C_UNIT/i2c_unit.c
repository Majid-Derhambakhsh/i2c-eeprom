/*
------------------------------------------------------------------------------
~ File   : i2c_unit.c
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

#include "i2c_unit.h"

/* ------------------ Functions ------------------ */

uint8_t I2C_Status(void) /* Function for take I2C status */
{
	
	return (_I2C_SR & _I2C_STATUS); /* Return I2C status */
	
}
/*
	Parameters    :
	
	Return Values :
					I2C Status.
	
	Example       :
					uint8_t twi_status;
			
					twi_status = I2C_Status();
			
*/

uint8_t I2C_BeginTransmission(uint16_t time_out) /* Function for send START condition */
{
	/* --------------------------- */
	
	_I2C_CR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN); /* Send START condition */
	
	/* --------------------------- */
	while ( (!(_I2C_CR & (1 << TWINT))) && (time_out > 0) ) /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	{
		_DELAY_MS(1);
		time_out--;
	}
	
	/* --------------------------- */
	return I2C_Status(); /* Return status */
	
}
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					I2C Status.
	
	Example       :
					uint8_t twi_status;
			
					twi_status = I2C_BeginTransmission(100);
			
*/

uint8_t I2C_Transmit(uint8_t data , uint16_t time_out) /* Function for transmit data */
{
	/* --------------------------- */
	_I2C_DR = data; /* Data to be transmitted */
	
	/* --------------------------- */
	_I2C_CR = (1 << TWINT) | (1 << TWEN); /* Start transmission */
	
	/* --------------------------- */
	while ( (!(_I2C_CR & (1 << TWINT))) && (time_out > 0)) /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	{
		_DELAY_MS(1);
		time_out--;
	}
	
	/* --------------------------- */
	return I2C_Status();
	
}
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

uint8_t I2C_ReceiveACK(uint16_t time_out) /* Function for receive data with ACK */
{
	/* --------------------------- */
	_I2C_CR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN); /* Acknowledge Send */
	
	/* --------------------------- */
	while ( (!(_I2C_CR & (1 << TWINT))) && (time_out > 0)) /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	{
		_DELAY_MS(1);
		time_out--;
	}
	
	/* --------------------------- */
	return _I2C_DR; /* Return received data */

}
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					Received data.
	
	Example       :
					uint8_t my_data;
			
					my_data = I2C_ReceiveACK(100);
			
*/

uint8_t I2C_ReceiveNACK(uint16_t time_out) /* Function for receive data with no ACK */
{
	/* --------------------------- */
	_I2C_CR = (1 << TWINT) | (1 << TWEN);

	/* --------------------------- */
	while ( (!(_I2C_CR & (1 << TWINT))) && (time_out > 0)) /* Wait for TWINT Flag set. This indicates that the START condition has been transmitted. */
	{
		_DELAY_MS(1);
		time_out--;
	}
	
	/* --------------------------- */
	return _I2C_DR; /* Return received data */
	
}
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
					Received data.
	
	Example       :
					uint8_t my_data;
			
					my_data = I2C_ReceiveNACK(100);
			
*/

void I2C_EndTransmission(uint16_t time_out) /* Function for stop transmission I2C */
{
	/* --------------------------- */
	_I2C_CR = ((1 << TWINT)|(1 << TWEN)|(1 << TWSTO)); /* Transmit STOP condition */

	/* --------------------------- */
	while ( (_I2C_CR & (1 << TWSTO)) && (time_out > 0)) /* Wait for TWSTO Flag reset. */
	{
		_DELAY_MS(1);
		time_out--;
	}
	
}
/*
	Parameters    :
					time_out : Timeout duration.
	
	Return Values :
	
	Example       :
					I2C_EndTransmission(250);
			
*/

void I2C_SetAddress(uint8_t address) /* Function for self I2C address */
{
	/* --------------------------- */
	_I2C_AR = address << 1; /* Set TWI slave address (upper 7 bits) */

}
/*
	Parameters    :
					address : Slave address of the TWI unit.
	
	Return Values :
	
	Example       :
					I2C_SetAddress(0x50);
			
*/

void I2C_Init(void) /* This Function is for Initialize the I2C peripheral. */
{
	
	/* ------- Initialize Prescaler & Bit rate ------- */
	
	_I2C_SR = _PRESCALER; /* Prescaler = 1 */
	_I2C_BR = (uint8_t)( ( (_F_CPU / _F_SCL) - 16 ) / (2 * _PRESCALER) ); /* Calculate and set i2c bit rate */

	/* ----------------------------------------------- */
	_I2C_CR = (1 << TWEN); /* Enable TWI  module */
	
}
/*
	Parameters    :
	
	Return Values :
	
	Example       :
					I2C_Init();
			
*/

void I2C_DeInit(void) /* This Function is for DeInitialize the I2C peripheral. */
{
	
	/* ------- Initialize Prescaler & Bit rate ------- */
	
	_I2C_SR = _I2C_DIS; /* Reset register */
	_I2C_BR = _I2C_DIS; /* Reset register */

	/* ----------------------------------------------- */
	_I2C_CR = _I2C_DIS; /* Reset register */
	
}
/*
	Parameters    :
	
	Return Values :
	
	Example       :
					I2C_DeInit();
			
*/

uint8_t I2C_IsDeviceReady(uint8_t dev_address , uint16_t trials , uint16_t time_out) /* Checks if target device is ready for communication. */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint16_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^ Reinitialize I2C ^^^^^^^^^^^^^^^^^^^^^ */
	
	I2C_DeInit(); /* DeInitialize i2c */
	I2C_Init(); /* Initialize i2c */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	for (; trials > 0 ; trials--)
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( dev_address , time_out); /* Send Device Address */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
			
		}
		else{}
		
	}
	
	/* --------------------------------- */
	
	if ( step_check > 0 ) /* The steps are complete */
	{
		i2c_status = _TRUE; /* Set status */
	}
	else
	{
		i2c_status = _FALSE; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

StatusTypeDef I2C_Master_Transmit(uint8_t dev_address , uint8_t *data , uint32_t quantity , uint16_t time_out) /* This function is for send data to i2c device */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint32_t write_quantity = quantity; /* Variable for check write quantity */
	uint32_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( I2C_IsDeviceReady(dev_address , 1 , time_out) == _TRUE ) /* target device is ready */
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( dev_address , time_out); /* Send Device Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_Transmit( *data , time_out ); /* Send first memory Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		for ( ; quantity > 0 ; quantity-- ) /* Loop for write data to device */
		{
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
			{
				
				data++; /* Select next byte */
				i2c_status = I2C_Transmit( *data , time_out ); /* Send data to device */
				step_check++; /* The step is completed */
				
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( step_check == (_MIN_STEP_FOR_DT + write_quantity) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else
		{
			i2c_status = _STAT_ERROR; /* Set status */
		}
		
	}
	else
	{
		i2c_status = _STAT_ERROR; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

StatusTypeDef I2C_Master_Receive(uint8_t dev_address , uint8_t *data , uint32_t quantity , uint16_t time_out) /* This function is for receive data from i2c device */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint32_t read_quantity = quantity; /* Variable for check read quantity */
	uint32_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( I2C_IsDeviceReady(dev_address , 1 , time_out) == _TRUE ) /* target device is ready */
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* ------------------ */
		
		if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( dev_address , time_out ); /* Send Device Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_BeginTransmission(time_out); /* Repeat Start */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_REP_START_TRANSMITTED ) /* A repeated START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( (dev_address | _DEVICE_READ) , time_out ); /* Send Device Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		for ( ; quantity > 1 ; quantity-- ) /* Loop for read data from device */
		{
			
			if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
			{
				
				*data = I2C_ReceiveACK(time_out); /* Receive Data with send ACK */
				i2c_status = I2C_Status(); /* I2C status take */
				step_check++; /* The step is completed */
				data++; /* Select next byte */
				
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
		{
			
			*data = I2C_ReceiveNACK(time_out); /* Receive Data with send NACK */
			i2c_status = I2C_Status(); /* I2C status take */
			step_check++; /* The step is completed */
			
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MR_DATA_RECEIVED_NACK ) /* Data byte has been received; NOT ACK has been returned */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( step_check == (_MIN_STEP_FOR_DR + read_quantity) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else
		{
			i2c_status = _STAT_ERROR; /* Set status */
		}
		
	}
	else
	{
		i2c_status = _STAT_ERROR; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

StatusTypeDef I2C_Mem_Write(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint8_t *mem_data , uint32_t quantity , uint16_t time_out) /* This function is for write data to external memory */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint32_t write_quantity = quantity; /* Variable for check write quantity */
	uint32_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( I2C_IsDeviceReady(dev_address , 1 , time_out) == _TRUE ) /* target device is ready */
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* --------------------------------- */
		
		if ( mem_add_size == _I2C_MEMADD_SIZE_8BIT ) /* Check memory address size */
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_8BIT) & _P0_BIT_SEL_MEMADD_SIZE_8BIT)) , time_out); /* Send Device Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		else
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_16BIT) & _P0_BIT_SEL_MEMADD_SIZE_16BIT)) , time_out); /* Send Device Address */
				step_check++; /* The step is completed */
			}
			else{}
				
			/* ~~~~~~~~~~~~ Send MSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)(mem_address >> _BYTE_SHIFT_VAL) , time_out); /* Send first memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		for ( ; quantity > 0 ; quantity-- ) /* Loop for write data to memory */
		{
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
			{
				
				i2c_status = I2C_Transmit( *mem_data , time_out ); /* Send data to memory */
				step_check++; /* The step is completed */
				mem_data++; /* Select next byte */
				
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( (mem_add_size == _I2C_MEMADD_SIZE_8BIT) && (step_check == ((_MEMORY_BURST_WRITE_STEPS - 1) + write_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else if ( (mem_add_size == _I2C_MEMADD_SIZE_16BIT) && (step_check == (_MEMORY_BURST_WRITE_STEPS + write_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else
		{
			i2c_status = _STAT_ERROR; /* Set status */
		}
		
	}
	else
	{
		i2c_status = _STAT_ERROR; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

StatusTypeDef I2C_Mem_Read(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint8_t *mem_data , uint32_t quantity , uint16_t time_out ) /* This function is for read data from external memory */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint32_t read_quantity = quantity; /* Variable for check read quantity */
	uint32_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( I2C_IsDeviceReady(dev_address , 1 , time_out) == _TRUE ) /* target device is ready */
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* --------------------------------- */
		
		if ( mem_add_size == _I2C_MEMADD_SIZE_8BIT ) /* Check memory address size */
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_8BIT) & _P0_BIT_SEL_MEMADD_SIZE_8BIT)) , time_out ); /* Send device Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		else
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_16BIT) & _P0_BIT_SEL_MEMADD_SIZE_16BIT)) , time_out ); /* Send device Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send MSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)(mem_address >> _BYTE_SHIFT_VAL) , time_out); /* Send first memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			i2c_status = I2C_BeginTransmission(time_out); /* Repeat Start */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_REP_START_TRANSMITTED ) /* A repeated START condition has been transmitted */
		{
			i2c_status = I2C_Transmit( (dev_address | _DEVICE_READ) , time_out ); /* Send device Address */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		for ( ; quantity > 1 ; quantity-- ) /* Loop for write data to register */
		{
			
			if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
			{
				
				*mem_data = I2C_ReceiveACK(time_out); /* Receive Data with send ACK */
				i2c_status = I2C_Status(); /* I2C status take */
				step_check++; /* The step is completed */
				mem_data++; /* Select next byte */
				
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( ( i2c_status == _MR_SLA_R_TRANSMITTED_ACK ) || ( i2c_status == _MR_DATA_RECEIVED_ACK ) ) /* SLA+R/DATA has been transmitted, and ACK has been received. */
		{
			
			*mem_data = I2C_ReceiveNACK(time_out); /* Receive Data with send NACK */
			i2c_status = I2C_Status(); /* I2C status take */
			step_check++; /* The step is completed */
			
		}
		else{}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MR_DATA_RECEIVED_NACK ) /* Data byte has been received; NOT ACK has been returned */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( (mem_add_size == _I2C_MEMADD_SIZE_8BIT) && (step_check == ((_MEMORY_BURST_READ_STEPS - 1) + read_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else if ( (mem_add_size == _I2C_MEMADD_SIZE_16BIT) && (step_check == (_MEMORY_BURST_READ_STEPS + read_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else
		{
			i2c_status = _STAT_ERROR; /* Set status */
		}
		
	}
	else
	{
		i2c_status = _STAT_ERROR; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

StatusTypeDef I2C_Mem_Erase(uint8_t dev_address , uint32_t mem_address , uint8_t mem_add_size , uint32_t quantity , uint16_t time_out) /* This function is for erase external memory */
{
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	uint8_t i2c_status = 0; /* Variable for check status */
	uint32_t write_quantity = quantity; /* Variable for check write quantity */
	uint32_t step_check = 0; /* Variable to check the completed steps */
	
	/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
	
	if ( I2C_IsDeviceReady(dev_address , 1 , time_out) == _TRUE ) /* target device is ready */
	{
		
		i2c_status = I2C_BeginTransmission(time_out); /* Begin Transmission */
		
		/* --------------------------------- */
		
		if ( mem_add_size == _I2C_MEMADD_SIZE_8BIT ) /* Check memory address size */
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_8BIT) & _P0_BIT_SEL_MEMADD_SIZE_8BIT)) , time_out); /* Send Device Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		else
		{
			
			if ( i2c_status == _MT_START_TRANSMITTED ) /* START condition has been transmitted */
			{
				i2c_status = I2C_Transmit( (dev_address | (uint8_t)((mem_address >> _P0_SHIFT_VAL_MEMADD_SIZE_16BIT) & _P0_BIT_SEL_MEMADD_SIZE_16BIT)) , time_out); /* Send Device Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send MSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_SLA_W_TRANSMITTED_ACK ) /* SLA+W has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)(mem_address >> _BYTE_SHIFT_VAL) , time_out); /* Send first memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
			/* ~~~~~~~~~~~~ Send LSB Memory Address ~~~~~~~~~~~~ */
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* Data has been transmitted, and ACK has been received. */
			{
				i2c_status = I2C_Transmit( (uint8_t)mem_address , time_out ); /* Send second memory Address */
				step_check++; /* The step is completed */
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		for ( ; quantity > 0 ; quantity-- ) /* Loop for write data to memory */
		{
			
			if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
			{
				
				i2c_status = I2C_Transmit( _MEMORY_DEF_VAL , time_out ); /* Send data to memory */
				step_check++; /* The step is completed */
				
			}
			else{}
			
		}
		
		/* --------------------------------- */
		
		if ( i2c_status == _MT_DATA_TRANSMITTED_ACK ) /* DATA has been transmitted, and ACK has been received. */
		{
			I2C_EndTransmission(time_out); /* End Transmission */
			step_check++; /* The step is completed */
		}
		else{}
		
		/* --------------------------------- */
		
		if ( (mem_add_size == _I2C_MEMADD_SIZE_8BIT) && (step_check == ((_MEMORY_BURST_WRITE_STEPS - 1) + write_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else if ( (mem_add_size == _I2C_MEMADD_SIZE_16BIT) && (step_check == (_MEMORY_BURST_WRITE_STEPS + write_quantity)) ) /* The steps are complete */
		{
			i2c_status = _STAT_OK; /* Set status */
		}
		else
		{
			i2c_status = _STAT_ERROR; /* Set status */
		}
		
	}
	else
	{
		i2c_status = _STAT_ERROR; /* Set status */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

/* Program End */
