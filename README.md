![Banner](Banner.jpg)

# i2c-eeprom
Driver for using Serial EEPROM Products (AT24C family) in AVR-ARM microcontrollers.

### Version : 0.0.1

- #### Type : Embedded Software.

- #### Support : All C/C++ compiler.

- #### Program Language : C

- #### Properties :

### Initialization and de-initialization functions:
```c++
I2C_Memory_Init();
```  

### Operation functions:
```c++  
I2C_MemoryIsReady(uint16_t time_out);  
I2C_Memory_SingleWrite(uint32_t address,uint8_t udata,uint16_t time_out);  
I2C_Memory_BurstWrite(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out);  
I2C_Memory_SingleRead(uint32_t address,uint8_t *udata,uint16_t time_out);  
I2C_Memory_BurstRead(uint32_t address,uint8_t *udata,uint32_t size,uint16_t time_out);  
I2C_Memory_Erase(uint32_t address,uint32_t quantity,uint16_t time_out);  
``` 
### Macros:
```c++  
#define _ChipCode  
#define _MEMORY_Ax_PIN_STATE 
#define _WP_DISABLE  
#define _WRITE_PROTECT_PORT  
#define _WRITE_PROTECT_PIN  
``` 

## How to use this library

### The I2C Memory library can be used as follows:
#### 1.  Add .h and source file in project.      
#### 2.  Config Chipset in 'i2c_memory_conf.h' header, for example:  
   * Options:  
      ```c++
      #define _AT24C04 // Chipset code
      
      #define _MEMORY_A0_PIN_STATE 0  // Address pin 0 state
      #define _MEMORY_A1_PIN_STATE 1  // Address pin 1 state
      #define _MEMORY_A2_PIN_STATE 0  // Address pin 2 state
      
      #define _WP_DISABLE  // Write Protect disable macro
      
      #define _WRITE_PROTECT_PORT  GPIOA  // Connected Port for 'Write Protect'
      #define _WRITE_PROTECT_PIN   9      // Connected Pin for 'Write Protect'
      
      ```
          
          
#### 3.  Config I2C and initialize it.          
     
#### 4.  Using initialize methods for initialize hardware and chipset, for example:  
```c++  
I2C_Memory_Init();
```  
#### 5.  Using operation methods, for example:  
```c++  
int main(void)
{
   
   I2C_Memory_Init();   // Initialize
      
   while()
   {
     I2C_Memory_SingleWrite(0, 'a', 100);
     
     if (I2C_MemoryIsReady(100) == 0)
     {
       // Memory error
     }
     
   }
}
   
``` 
    
## Supported Chipset:
- [x] AT24 Series
- [ ] AT25 Series 

#### Developer: Majid Derhambakhsh
