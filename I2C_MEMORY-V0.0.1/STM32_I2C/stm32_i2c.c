/*
------------------------------------------------------------------------------
~ File   : stm32_i2c.c
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

#include "stm32_i2c.h"

/* ------------------ Functions ------------------ */

HAL_StatusTypeDef HAL_I2C_Mem_Write2(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) /* This function is for write data to external memory */
{

	/* -------------------------------------------- */
	
	HAL_StatusTypeDef i2c_status; /* Create enum */
	
	/* -------------------------------------------- */
	
	if ( MemAddSize == I2C_MEMADD_SIZE_8BIT ) /* Check memory address size */
	{
		i2c_status = HAL_I2C_Mem_Write(hi2c , _CALC_DEVADD_8BIT(DevAddress,MemAddress) , MemAddress , MemAddSize , pData , Size , Timeout); /* Send Data */
	}
	else
	{
		i2c_status = HAL_I2C_Mem_Write(hi2c , _CALC_DEVADD_16BIT(DevAddress,MemAddress) , MemAddress , MemAddSize , pData , Size , Timeout); /* Send Data */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

HAL_StatusTypeDef HAL_I2C_Mem_Read2(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) /* This function is for read data from external memory */
{

	/* -------------------------------------------- */
	
	HAL_StatusTypeDef i2c_status; /* Create enum */
	
	/* ............................................ */
	
	if ( MemAddSize == I2C_MEMADD_SIZE_8BIT ) /* Check memory address size */
	{
		i2c_status = HAL_I2C_Mem_Read(hi2c , _CALC_DEVADD_8BIT(DevAddress,MemAddress) , MemAddress , MemAddSize , pData , Size , Timeout); /* Receive Data */
	}
	else
	{
		i2c_status = HAL_I2C_Mem_Read(hi2c , _CALC_DEVADD_16BIT(DevAddress,MemAddress) , MemAddress , MemAddSize , pData , Size , Timeout); /* Receive Data */
	}
	
	return i2c_status;
	
	/* Function End */
}
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

HAL_StatusTypeDef HAL_I2C_Mem_Erase(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint16_t Size, uint32_t stwc, uint32_t Timeout) /* This function is for erase external memory */
{
	
	/* ------------------- Variable ------------------- */
	
	uint8_t mem_def_val_set = 0; /* Memory default value counter */
	uint8_t memory_def[_MEM_DEF_VAL_BUFF_LENGTH]; /* Memory default value */
	
	/* --------------------- Enum --------------------- */
	
	HAL_StatusTypeDef i2c_status = HAL_OK; /* Create enum */
	
	/* ................................................ */
	
	for (mem_def_val_set = 0 ; mem_def_val_set < _MEM_DEF_VAL_BUFF_LENGTH ; mem_def_val_set++ ) /* Loop for set value */
	{
		memory_def[mem_def_val_set] = _MEMORY_DEF_VAL; /* Set value */
	}
	
	/* ................................................ */
	
	if (Size <= _MEM_DEF_VAL_BUFF_LENGTH) /* Check size */
	{
		
		i2c_status = HAL_I2C_Mem_Write2(hi2c,DevAddress,MemAddress,MemAddSize,memory_def,Size,Timeout); /* Write to memory */
		HAL_Delay(stwc); /* Delay for Self Timed Write Cycle */
		
	}
	else
	{
		
		while ( (Size > 0) && (i2c_status == HAL_OK) ) /* Check status */
		{
			
			if (Size >= _MEM_DEF_VAL_BUFF_LENGTH) /* Check size */
			{
				
				i2c_status = HAL_I2C_Mem_Write2(hi2c,DevAddress,MemAddress,MemAddSize,memory_def,_MEM_DEF_VAL_BUFF_LENGTH,Timeout); /* Write to memory */
				
				MemAddress += _MEM_DEF_VAL_BUFF_LENGTH; /* Set new address */
				Size       -= _MEM_DEF_VAL_BUFF_LENGTH; /* Set new size */
				
			}
			else
			{
				
				i2c_status = HAL_I2C_Mem_Write2(hi2c,DevAddress,MemAddress,MemAddSize,memory_def,Size,Timeout);  /* Write to memory */
				
				Size = 0; /* Reset value */
				
			}
			
			HAL_Delay(stwc); /* Delay for Self Timed Write Cycle */
			
		}
		
	}
	
	return i2c_status;
	
	/* Function End */
}
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

/* Program End */
