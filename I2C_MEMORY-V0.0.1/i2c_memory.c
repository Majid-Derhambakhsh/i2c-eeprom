/*
------------------------------------------------------------------------------
~ File   : i2c_memory.c
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

#include "i2c_memory.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

void I2C_Memory_Init(void) /* This function is for initialize memory */
{
	
	#ifndef _WP_DISABLE
	
	_MEM_WP_CONTROL(_GPIO_PIN_SET); /* Enable Write Protect */
	
	#endif /* _WP_DISABLE */
	
}
/*
	Guide   :
			 Parameters    :
			 
			 Return Values :
			
	Example :
			 I2C_Memory_Init();
	
*/

uint8_t I2C_MemoryIsReady(uint16_t time_out) /* Checks if memory is ready for communication */
{
	return _I2C_MEM_READY(_DEV_ADDRESS,_TRIALS,time_out); /* Check device */
}
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

uint8_t I2C_Memory_SingleWrite(uint32_t address,uint8_t udata,uint16_t time_out) /* This function is for write data to memory */
{
	
	/* ------------ Variable ------------- */
	
	uint8_t com_status = _ERROR_VAL; /* Variable for take status */
	
	/* ................................... */
	
	if (address < _MEM_MAX_SIZE) /* Check Address */
	{
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_RESET); /* Disable Write Protect */
		#endif /* _WP_DISABLE */
		
		/* ................................... */
		
		com_status = _I2C_MEM_WRITE(_DEV_ADDRESS,address,_MEMADD_SIZE,&udata,1,time_out); /* Write data */
		_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
		
		/* ................................... */
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_SET);  /* Enable Write Protect */
		#endif /* _WP_DISABLE */
		
	}
	else
	{
		com_status = _MEM_SIZE_ERROR; /* Set status */
	}
	
	return com_status;
	
	/* Function End */
}
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

uint8_t I2C_Memory_BurstWrite(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out) /* This function is for write data to memory */
{
	
	/* ------------ Variable ------------- */
	
	uint8_t   com_status    = _ERROR_VAL; /* Variable for take status */
	uint16_t  start_page    = address / _MEM_PAGE_SIZE; /* Calculating memory starting page */
	uint16_t  end_page      = (address + size) / _MEM_PAGE_SIZE; /* Calculating memory end page */
	uint16_t  page_capacity = ((start_page + 1) * _MEM_PAGE_SIZE) - address; /* Calculating memory page capacity */
	
	/* ................................... */
	
	if ( (address + size) <= _MEM_MAX_SIZE ) /* Check data length */
	{
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_RESET); /* Disable Write Protect */
		#endif /* _WP_DISABLE */
		
		/* ................................... */
		
		if (page_capacity >= size) /* Check memory page capacity */
		{
			
			com_status = _I2C_MEM_WRITE(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,size,time_out); /* Write data */
			_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
			
		}
		else
		{
			
			com_status = _I2C_MEM_WRITE(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,page_capacity,time_out); /* Write data */
			_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
			
			/* ..................................... */
			
			if (com_status == _OK_VAL) /* Check i2c status */
			{
				
				address += page_capacity; /* Set new address */
				udata   += page_capacity; /* Set new data address */
				size    -= page_capacity; /* Set new data length */
				
				start_page++; /* Increase page number */
				
				/* ..................................... */
				
				for (; (start_page <= end_page) && (com_status == _OK_VAL) ;start_page++) /* Loop for write data */
				{
					
					if (size < _MEM_PAGE_SIZE) /* Check data length */
					{
						com_status = _I2C_MEM_WRITE(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,size,time_out); /* Write data */
					}
					else
					{
						
						com_status = _I2C_MEM_WRITE(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,_MEM_PAGE_SIZE,time_out); /* Write data */
						
						address += _MEM_PAGE_SIZE; /* Set new address */
						udata   += _MEM_PAGE_SIZE; /* Set new data address */
						size    -= _MEM_PAGE_SIZE; /* Set new data length */
						
					}
					
					_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
					
				}
				
			}
			
		}
		
		/* ................................... */
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_SET); /* Enable Write Protect */
		#endif /* _WP_DISABLE */
		
	}
	else
	{
		com_status = _MEM_SIZE_ERROR; /* Set status */
	}
	
	return com_status;
	
	/* Function End */
}
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

uint8_t I2C_Memory_SingleRead(uint32_t address,uint8_t *udata,uint16_t time_out) /* This function is for read data from memory */
{
	
	/* ------------ Variable ------------- */
	
	uint8_t com_status = _ERROR_VAL; /* Variable for take status */
	
	/* ................................... */
	
	if (address < _MEM_MAX_SIZE) /* Check Address */
	{
		
		com_status = _I2C_MEM_READ(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,1,time_out); /* Write data */
		_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
		
	}
	else
	{
		com_status = _MEM_SIZE_ERROR; /* Set status */
	}
	
	return com_status;
	
	/* Function End */
}
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

uint8_t I2C_Memory_BurstRead(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out) /* This function is for read data from memory */
{
	
	/* ------------ Variable ------------- */
	
	uint8_t com_status = _ERROR_VAL; /* Variable for take status */
	
	/* ................................... */
	
	if ( (address + size) <= _MEM_MAX_SIZE ) /* Check data length */
	{
		
		com_status = _I2C_MEM_READ(_DEV_ADDRESS,address,_MEMADD_SIZE,udata,size,time_out); /* Write data */
		_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
		
	}
	else
	{
		com_status = _MEM_SIZE_ERROR; /* Set status */
	}
	
	return com_status;
	
	/* Function End */
}
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

uint8_t I2C_Memory_Erase(uint32_t address,uint32_t quantity,uint16_t time_out) /* This function is for erase data of memory */
{
	
	/* ------------ Variable ------------- */
	
	uint8_t   com_status    = _ERROR_VAL; /* Variable for take status */
	uint16_t  start_page    = address / _MEM_PAGE_SIZE; /* Calculating memory starting page */
	uint16_t  end_page      = (address + quantity) / _MEM_PAGE_SIZE; /* Calculating memory end page */
	uint16_t  page_capacity = ((start_page + 1) * _MEM_PAGE_SIZE) - address; /* Calculating memory page capacity */
	
	/* ................................... */
	
	if ( (address + quantity) <= _MEM_MAX_SIZE ) /* Check data length */
	{
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_RESET); /* Disable Write Protect */
		#endif /* _WP_DISABLE */
		
		/* ................................... */
		
		if (page_capacity >= quantity) /* Check memory page capacity */
		{
			
			com_status = _I2C_MEM_ERASE(_DEV_ADDRESS,address,_MEMADD_SIZE,quantity,time_out); /* Write data */
			_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
			
		}
		else
		{
			
			com_status = _I2C_MEM_ERASE(_DEV_ADDRESS,address,_MEMADD_SIZE,page_capacity,time_out); /* Write data */
			_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
			
			/* ..................................... */
			
			if (com_status == _OK_VAL) /* Check i2c status */
			{
				
				address   += page_capacity; /* Set new address */
				quantity  -= page_capacity; /* Set new erase quantity */
				
				start_page++; /* Increase page number */
				
				/* ..................................... */
				
				for (; (start_page <= end_page) && (com_status == _OK_VAL) ;start_page++) /* Loop for write data */
				{
					
					if (quantity < _MEM_PAGE_SIZE) /* Check erase quantity */
					{
						com_status = _I2C_MEM_ERASE(_DEV_ADDRESS,address,_MEMADD_SIZE,quantity,time_out); /* Write data */
					}
					else
					{
						
						com_status = _I2C_MEM_ERASE(_DEV_ADDRESS,address,_MEMADD_SIZE,_MEM_PAGE_SIZE,time_out); /* Write data */
						
						address   += _MEM_PAGE_SIZE; /* Set new address */
						quantity  -= _MEM_PAGE_SIZE; /* Set new erase quantity */
						
					}
					
					_DELAY_MS(_MEM_STWC); /* Delay for Self Timed Write Cycle */
					
				}
				
			}
			
		}
		
		/* ................................... */
		
		#ifndef _WP_DISABLE
			_MEM_WP_CONTROL(_GPIO_PIN_SET); /* Enable Write Protect */
		#endif /* _WP_DISABLE */
		
	}
	else
	{
		com_status = _MEM_SIZE_ERROR; /* Set status */
	}
	
	return com_status;
	
	/* Function End */
}
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
