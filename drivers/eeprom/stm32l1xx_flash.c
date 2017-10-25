/**
  ******************************************************************************
  * @file    stm32l1xx_flash.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    05-March-2012
  * @brief   This file provides all the Flash firmware functions. These functions 
  *          can be executed from Internal FLASH or Internal SRAM memories. 
  *          The functions that should be called from SRAM are defined inside 
  *          the "stm32l1xx_flash_ramfunc.c" file.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the FLASH peripheral:
  *            + FLASH Interface configuration
  *            + FLASH Memory Programming
  *            + DATA EEPROM Programming
  *            + Option Bytes Programming
  *            + Interrupts and flags management
  *
  *  @verbatim

  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..] This driver provides functions to configure and program the Flash 
         memory of all STM32L1xx devices.
    [..] These functions are split in 5 groups:
         (#) FLASH Interface configuration functions: this group includes 
             the management of following features:
             (++) Set the latency.
             (++) Enable/Disable the prefetch buffer.
             (++) Enable/Disable the 64 bit Read Access. 
             (++) Enable/Disable the RUN PowerDown mode.
             (++) Enable/Disable the SLEEP PowerDown mode.  
    
         (#) FLASH Memory Programming functions: this group includes all 
             needed functions to erase and program the main memory:
             (++) Lock and Unlock the Flash interface.
             (++) Erase function: Erase Page.
             (++) Program functions: Fast Word and Half Page(should be 
                  executed from internal SRAM).
      
         (#) DATA EEPROM Programming functions: this group includes all 
             needed functions to erase and program the DATA EEPROM memory:
             (++) Lock and Unlock the DATA EEPROM interface.
             (++) Erase function: Erase Byte, erase HalfWord, erase Word, erase 
             (++) Double Word (should be executed from internal SRAM).
             (++) Program functions: Fast Program Byte, Fast Program Half-Word, 
                  FastProgramWord, Program Byte, Program Half-Word, 
                  Program Word and Program Double-Word (should be executed 
                  from internal SRAM).
      
         (#) FLASH Option Bytes Programming functions: this group includes 
             all needed functions to:
             (++) Lock and Unlock the Flash Option bytes.
             (++) Set/Reset the write protection.
             (++) Set the Read protection Level.
             (++) Set the BOR level.
             (++) rogram the user option Bytes.
             (++) Launch the Option Bytes loader.
             (++) Get the Write protection.
             (++) Get the read protection status.
             (++) Get the BOR level.
             (++) Get the user option bytes.
    
         (#) FLASH Interrupts and flag management functions: this group 
             includes all needed functions to:
             (++) Enable/Disable the flash interrupt sources.
             (++) Get flags status.
             (++) Clear flags.
             (++) Get Flash operation status.
             (++) Wait for last flash operation.

  *  @endverbatim
  *                      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "board.h"

#ifdef STM32L1XX_MD
/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32l1xx_flash.h"

/** @addtogroup STM32L1xx_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH 
  * @brief FLASH driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
  
/* FLASH Mask */
#define WRP01_MASK                 ((uint32_t)0x0000FFFF)
#define WRP23_MASK                 ((uint32_t)0xFFFF0000)
#define WRP45_MASK                 ((uint32_t)0x0000FFFF)
#define WRP67_MASK                 ((uint32_t)0xFFFF0000)
#define WRP89_MASK                 ((uint32_t)0x0000FFFF)
#define WRP1011_MASK               ((uint32_t)0xFFFF0000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
 
/** @defgroup FLASH_Private_Functions
  * @{
  */ 

/** @defgroup FLASH_Group1 FLASH Interface configuration functions
  *  @brief   FLASH Interface configuration functions 
 *
@verbatim   
  ============================================================================== 
             ##### FLASH Interface configuration functions #####
  ==============================================================================

    [..] FLASH_Interface configuration_Functions, includes the following functions:
     (+) void FLASH_SetLatency(uint32_t FLASH_Latency):
    [..] To correctly read data from Flash memory, the number of wait states (LATENCY) 
         must be correctly programmed according to the frequency of the CPU clock 
        (HCLK) and the supply voltage of the device.
  [..] 
  ----------------------------------------------------------------
 |  Wait states  |                HCLK clock frequency (MHz)      |
 |               |------------------------------------------------|
 |   (Latency)   |            voltage range       | voltage range |
 |               |            1.65 V - 3.6 V      | 2.0 V - 3.6 V |
 |               |----------------|---------------|---------------|
 |               |  VCORE = 1.2 V | VCORE = 1.5 V | VCORE = 1.8 V |
 |-------------- |----------------|---------------|---------------|
 |0WS(1CPU cycle)|0 < HCLK <= 2   |0 < HCLK <= 8  |0 < HCLK <= 16 |
 |---------------|----------------|---------------|---------------|
 |1WS(2CPU cycle)|2 < HCLK <= 4   |8 < HCLK <= 16 |16 < HCLK <= 32|
  ----------------------------------------------------------------
  [..]
     (+) void FLASH_PrefetchBufferCmd(FunctionalState NewState);
     (+) void FLASH_ReadAccess64Cmd(FunctionalState NewState);
     (+) void FLASH_RUNPowerDownCmd(FunctionalState NewState);
     (+) void FLASH_SLEEPPowerDownCmd(FunctionalState NewState);
     (+) void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
  [..]     
  Here below the allowed configuration of Latency, 64Bit access and prefetch buffer
  [..]  
  --------------------------------------------------------------------------------
 |               |              ACC64 = 0         |              ACC64 = 1        |
 |   Latency     |----------------|---------------|---------------|---------------|
 |               |   PRFTEN = 0   |   PRFTEN = 1  |   PRFTEN = 0  |   PRFTEN = 1  |
 |---------------|----------------|---------------|---------------|---------------|
 |0WS(1CPU cycle)|     YES        |     NO        |     YES       |     YES       |
 |---------------|----------------|---------------|---------------|---------------|
 |1WS(2CPU cycle)|     NO         |     NO        |     YES       |     YES       |
  --------------------------------------------------------------------------------
  [..]
   All these functions don't need the unlock sequence.

@endverbatim
  * @{
  */

/**
  * @brief  Sets the code latency value.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *   This parameter can be one of the following values:
  *     @arg FLASH_Latency_0: FLASH Zero Latency cycle.
  *     @arg FLASH_Latency_1: FLASH One Latency cycle.
  * @retval None
  */
void FLASH_SetLatency(uint32_t FLASH_Latency)
{
   uint32_t tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_FLASH_LATENCY(FLASH_Latency));
  
  /* Read the ACR register */
  tmpreg = FLASH->ACR;  
  
  /* Sets the Latency value */
  tmpreg &= (uint32_t) (~((uint32_t)FLASH_ACR_LATENCY));
  tmpreg |= FLASH_Latency;
  
  /* Write the ACR register */
  FLASH->ACR = tmpreg;
}

/**
  * @brief  Enables or disables the Prefetch Buffer.
  * @param  NewState: new state of the FLASH prefetch buffer.
  *              This parameter can be: ENABLE or DISABLE. 
  * @retval None
  */
void FLASH_PrefetchBufferCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
   
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_PRFTEN;
  }
  else
  {
    FLASH->ACR &= (uint32_t)(~((uint32_t)FLASH_ACR_PRFTEN));
  }
}

/**
  * @brief  Enables or disables read access to flash by 64 bits.
  * @param  NewState: new state of the FLASH read access mode.
  *              This parameter can be: ENABLE or DISABLE.
  * @note    If this bit is set, the Read access 64 bit is used.
  *          If this bit is reset, the Read access 32 bit is used.
  * @note    This bit cannot be written at the same time as the LATENCY and 
  *          PRFTEN bits.
  *          To reset this bit, the LATENCY should be zero wait state and the 
  *          prefetch off.
  * @retval None
  */
void FLASH_ReadAccess64Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_ACC64;
  }
  else
  {
    FLASH->ACR &= (uint32_t)(~((uint32_t)FLASH_ACR_ACC64));
  }
}

/**
  * @brief  Enable or disable the power down mode during Sleep mode.
  * @note   This function is used to power down the FLASH when the system is in SLEEP LP mode.
  * @param  NewState: new state of the power down mode during sleep mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void FLASH_SLEEPPowerDownCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Set the SLEEP_PD bit to put Flash in power down mode during sleep mode */
    FLASH->ACR |= FLASH_ACR_SLEEP_PD;
  }
  else
  {
    /* Clear the SLEEP_PD bit in to put Flash in idle mode during sleep mode */
    FLASH->ACR &= (uint32_t)(~((uint32_t)FLASH_ACR_SLEEP_PD));
  }
}

/**
  * @}
  */

/** @defgroup FLASH_Group2 FLASH Memory Programming functions
 *  @brief   FLASH Memory Programming functions
 *
@verbatim   
  ==============================================================================
                ##### FLASH Memory Programming functions ##### 
  ==============================================================================

    [..] The FLASH Memory Programming functions, includes the following functions:
    (+) void FLASH_Unlock(void);
    (+) void FLASH_Lock(void);
    (+) FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
    (+) FLASH_Status FLASH_FastProgramWord(uint32_t Address, uint32_t Data);
   
    [..] Any operation of erase or program should follow these steps:
    (#) Call the FLASH_Unlock() function to enable the flash control register and 
        program memory access.
    (#) Call the desired function to erase page or program data.
    (#) Call the FLASH_Lock() to disable the flash program memory access 
       (recommended to protect the FLASH memory against possible unwanted operation).

@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the FLASH control register and program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->PECR & FLASH_PECR_PRGLOCK) != RESET)
  {
    /* Unlocking the data memory and FLASH_PECR register access */
    DATA_EEPROM_Unlock();
  
    /* Unlocking the program memory access */
    FLASH->PRGKEYR = FLASH_PRGKEY1;
    FLASH->PRGKEYR = FLASH_PRGKEY2;  
  }
}

/**
  * @brief  Locks the Program memory access.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the PRGLOCK Bit to lock the program memory access */
  FLASH->PECR |= FLASH_PECR_PRGLOCK;
}

/**
  * @brief  Erases a specified page in program memory.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  *         Call the FLASH_Lock() to disable the flash memory access 
  *         (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  Page_Address: The page address in program memory to be erased.
  * @note   A Page is erased in the Program memory only if the address to load 
  *         is the start address of a page (multiple of 256 bytes).
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Page_Address));
 
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to erase the page */

    /* Set the ERASE bit */
    FLASH->PECR |= FLASH_PECR_ERASE;

    /* Set PROG bit */
    FLASH->PECR |= FLASH_PECR_PROG;
  
    /* Write 00000000h to the first word of the program page to erase */
    *(__IO uint32_t *)Page_Address = 0x00000000;
 
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    /* If the erase operation is completed, disable the ERASE and PROG bits */
    FLASH->PECR &= (uint32_t)(~FLASH_PECR_PROG);
    FLASH->PECR &= (uint32_t)(~FLASH_PECR_ERASE);   
  }     
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Programs a word at a specified address in program memory.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  *         Call the FLASH_Lock() to disable the flash memory access
  *         (recommended to protect the FLASH memory against possible unwanted operation).
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @retval FLASH Status: The returned value can be:  
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_FastProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* If the previous operation is completed, proceed to program the new  word */  
    *(__IO uint32_t *)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);       
  }
  /* Return the Write Status */
  return status;
}

/**
  * @}
  */
  
/** @defgroup FLASH_Group3 DATA EEPROM Programming functions
 *  @brief   DATA EEPROM Programming functions
 *
@verbatim   
 ===============================================================================
                     ##### DATA EEPROM Programming functions ##### 
 ===============================================================================  
 
    [..] The DATA_EEPROM Programming_Functions, includes the following functions:
        (+) void DATA_EEPROM_Unlock(void);
        (+) void DATA_EEPROM_Lock(void);
        (+) FLASH_Status DATA_EEPROM_EraseByte(uint32_t Address);
        (+) FLASH_Status DATA_EEPROM_EraseHalfWord(uint32_t Address);
        (+) FLASH_Status DATA_EEPROM_EraseWord(uint32_t Address);
        (+) FLASH_Status DATA_EEPROM_FastProgramByte(uint32_t Address, uint8_t Data);
        (+) FLASH_Status DATA_EEPROM_FastProgramHalfWord(uint32_t Address, uint16_t Data);
        (+) FLASH_Status DATA_EEPROM_FastProgramWord(uint32_t Address, uint32_t Data);
        (+) FLASH_Status DATA_EEPROM_ProgramByte(uint32_t Address, uint8_t Data);
        (+) FLASH_Status DATA_EEPROM_ProgramHalfWord(uint32_t Address, uint16_t Data);
        (+) FLASH_Status DATA_EEPROM_ProgramWord(uint32_t Address, uint32_t Data);
   
    [..] Any operation of erase or program should follow these steps:
    (#) Call the DATA_EEPROM_Unlock() function to enable the data EEPROM access
        and Flash program erase control register access.
    (#) Call the desired function to erase or program data.
    (#) Call the DATA_EEPROM_Lock() to disable the data EEPROM access
        and Flash program erase control register access(recommended
        to protect the DATA_EEPROM against possible unwanted operation).

@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the data memory and FLASH_PECR register access.
  * @param  None
  * @retval None
  */
void DATA_EEPROM_Unlock(void)
{
  if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
  {  
    /* Unlocking the Data memory and FLASH_PECR register access*/
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }
}

/**
  * @brief  Locks the Data memory and FLASH_PECR register access.
  * @param  None
  * @retval None
  */
void DATA_EEPROM_Lock(void)
{
  /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
  FLASH->PECR |= FLASH_PECR_PELOCK;
}

/**
  * @brief  Enables or disables DATA EEPROM fixed Time programming (2*Tprog).
  * @param  NewState: new state of the DATA EEPROM fixed Time programming mode.
  *         This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
void DATA_EEPROM_FixedTimeProgramCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    FLASH->PECR |= (uint32_t)FLASH_PECR_FTDW;
  }
  else
  {
    FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FTDW));
  }
}

/**
  * @brief  Erase a byte in data memory.
  * @param  Address: specifies the address to be erased.
  * @note   This function can be used only for STM32L1XX_HD and STM32L1XX_MDP 
  *         density devices.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status DATA_EEPROM_EraseByte(uint32_t Address)
{
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* Write "00h" to valid address in the data memory" */
    *(__IO uint8_t *) Address = (uint8_t)0x00;
  }
   
  /* Return the erase status */
  return status;
}

/**
  * @brief  Erase a halfword in data memory.
  * @param  Address: specifies the address to be erased.
  * @note   This function can be used only for STM32L1XX_HD and STM32L1XX_MDP 
  *         density devices.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status DATA_EEPROM_EraseHalfWord(uint32_t Address)
{
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* Write "0000h" to valid address in the data memory" */
    *(__IO uint16_t *) Address = (uint16_t)0x0000;
  }
   
  /* Return the erase status */
  return status;
}

/**
  * @brief  Erase a word in data memory.
  * @param  Address: specifies the address to be erased.
  * @note   For STM32L1XX_MD, A data memory word is erased in the data memory only 
  *         if the address to load is the start address of a word (multiple of a word).
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status DATA_EEPROM_EraseWord(uint32_t Address)
{
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* Write "00000000h" to valid address in the data memory" */
    *(__IO uint32_t *) Address = 0x00000000;
  }
   
  /* Return the erase status */
  return status;
}

/**
  * @brief  Write a Byte at a specified address in data memory.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @note   This function assumes that the is data word is already erased.
  * @retval FLASH Status: The returned value can be:
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status DATA_EEPROM_FastProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
  uint32_t tmp = 0, tmpaddr = 0;
#endif
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address)); 

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
  if(status == FLASH_COMPLETE)
  {
    /* Clear the FTDW bit */
    FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FTDW));

#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
    if(Data != (uint8_t)0x00) 
    {
      /* If the previous operation is completed, proceed to write the new Data */
      *(__IO uint8_t *)Address = Data;
            
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    }
    else
    {
      tmpaddr = Address & 0xFFFFFFFC;
      tmp = * (__IO uint32_t *) tmpaddr;
      tmpaddr = 0xFF << ((uint32_t) (0x8 * (Address & 0x3)));
      tmp &= ~tmpaddr;
      status = DATA_EEPROM_EraseWord(Address & 0xFFFFFFFC);
      status = DATA_EEPROM_FastProgramWord((Address & 0xFFFFFFFC), tmp);
    }       
#elif defined (STM32L1XX_HD) || defined (STM32L1XX_MDP)
    /* If the previous operation is completed, proceed to write the new Data */
    *(__IO uint8_t *)Address = Data;
            
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
#endif  
  }
  /* Return the Write Status */
  return status;
}

/**
  * @brief  Writes a half word at a specified address in data memory.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @note   This function assumes that the is data word is already erased.
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or  FLASH_TIMEOUT. 
  */
FLASH_Status DATA_EEPROM_FastProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
  uint32_t tmp = 0, tmpaddr = 0;
#endif
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
  if(status == FLASH_COMPLETE)
  {
    /* Clear the FTDW bit */
    FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FTDW));

#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
    if(Data != (uint16_t)0x0000) 
    {
      /* If the previous operation is completed, proceed to write the new data */
      *(__IO uint16_t *)Address = Data;
  
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    }
    else
    {
      if((Address & 0x3) != 0x3)
      {
        tmpaddr = Address & 0xFFFFFFFC;
        tmp = * (__IO uint32_t *) tmpaddr;
        tmpaddr = 0xFFFF << ((uint32_t) (0x8 * (Address & 0x3)));
        tmp &= ~tmpaddr;        
        status = DATA_EEPROM_EraseWord(Address & 0xFFFFFFFC);
        status = DATA_EEPROM_FastProgramWord((Address & 0xFFFFFFFC), tmp);
      }
      else
      {
        DATA_EEPROM_FastProgramByte(Address, 0x00);
        DATA_EEPROM_FastProgramByte(Address + 1, 0x00);
      }
    }
#elif defined (STM32L1XX_HD) || defined (STM32L1XX_MDP)
    /* If the previous operation is completed, proceed to write the new data */
    *(__IO uint16_t *)Address = Data;
  
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
#endif
  }
  /* Return the Write Status */
  return status;
}

/**
  * @brief  Programs a word at a specified address in data memory.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to the data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @note   This function assumes that the is data word is already erased.
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status DATA_EEPROM_FastProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    /* Clear the FTDW bit */
    FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FTDW));
  
    /* If the previous operation is completed, proceed to program the new data */    
    *(__IO uint32_t *)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);       
  }
  /* Return the Write Status */
  return status;
}

/**
  * @brief  Write a Byte at a specified address in data memory without erase.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @note   The function  DATA_EEPROM_FixedTimeProgramCmd() can be called before 
  *         this function to configure the Fixed Time Programming.
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status DATA_EEPROM_ProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
  uint32_t tmp = 0, tmpaddr = 0;
#endif
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address)); 

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
    if(Data != (uint8_t) 0x00)
    {  
      *(__IO uint8_t *)Address = Data;
    
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

    }
    else
    {
      tmpaddr = Address & 0xFFFFFFFC;
      tmp = * (__IO uint32_t *) tmpaddr;
      tmpaddr = 0xFF << ((uint32_t) (0x8 * (Address & 0x3)));
      tmp &= ~tmpaddr;        
      status = DATA_EEPROM_EraseWord(Address & 0xFFFFFFFC);
      status = DATA_EEPROM_FastProgramWord((Address & 0xFFFFFFFC), tmp);
    }
#elif defined (STM32L1XX_HD) || defined (STM32L1XX_MDP)
    *(__IO uint8_t *)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
#endif
  }
  /* Return the Write Status */
  return status;
}

/**
  * @brief  Writes a half word at a specified address in data memory without erase.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @note   The function  DATA_EEPROM_FixedTimeProgramCmd() can be called before 
  *         this function to configure the Fixed Time Programming
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @retval FLASH Status: The returned value can be:
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status DATA_EEPROM_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
  uint32_t tmp = 0, tmpaddr = 0;
#endif
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
#if !defined (STM32L1XX_HD) && !defined (STM32L1XX_MDP)
    if(Data != (uint16_t)0x0000)
    {
      *(__IO uint16_t *)Address = Data;
   
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    }
    else
    {
      if((Address & 0x3) != 0x3)
      {
        tmpaddr = Address & 0xFFFFFFFC;
        tmp = * (__IO uint32_t *) tmpaddr;
        tmpaddr = 0xFFFF << ((uint32_t) (0x8 * (Address & 0x3)));
        tmp &= ~tmpaddr;          
        status = DATA_EEPROM_EraseWord(Address & 0xFFFFFFFC);
        status = DATA_EEPROM_FastProgramWord((Address & 0xFFFFFFFC), tmp);
      }
      else
      {
        DATA_EEPROM_FastProgramByte(Address, 0x00);
        DATA_EEPROM_FastProgramByte(Address + 1, 0x00);
      }
    }
#elif defined (STM32L1XX_HD) || defined (STM32L1XX_MDP)
    *(__IO uint16_t *)Address = Data;
   
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
#endif
  }
  /* Return the Write Status */
  return status;
}

/**
  * @brief  Programs a word at a specified address in data memory without erase.
  * @note   To correctly run this function, the DATA_EEPROM_Unlock() function
  *         must be called before.
  *         Call the DATA_EEPROM_Lock() to he data EEPROM access
  *         and Flash program erase control register access(recommended to protect 
  *         the DATA_EEPROM against possible unwanted operation).
  * @note   The function  DATA_EEPROM_FixedTimeProgramCmd() can be called before 
  *         this function to configure the Fixed Time Programming.
  * @param  Address: specifies the address to be written.
  * @param  Data: specifies the data to be written.
  * @retval FLASH Status: The returned value can be:
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or  FLASH_TIMEOUT. 
  */
FLASH_Status DATA_EEPROM_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_FLASH_DATA_ADDRESS(Address));
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
  if(status == FLASH_COMPLETE)
  {
    *(__IO uint32_t *)Address = Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  }
  /* Return the Write Status */
  return status;
}
/**
  * @}
  */

/** @defgroup FLASH_Group5 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
  ==============================================================================
              ##### Interrupts and flags management functions #####
  ==============================================================================    

@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified FLASH interrupts.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or 
  *         disabled.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: FLASH end of programming Interrupt
  *     @arg FLASH_IT_ERR: FLASH Error Interrupt
  * @retval None 
  */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FLASH_IT(FLASH_IT)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    /* Enable the interrupt sources */
    FLASH->PECR |= FLASH_IT;
  }
  else
  {
    /* Disable the interrupt sources */
    FLASH->PECR &= ~(uint32_t)FLASH_IT;
  }
}

/**
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *   This parameter can be one of the following values:
  *     @arg FLASH_FLAG_BSY: FLASH write/erase operations in progress flag 
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
  *     @arg FLASH_FLAG_READY: FLASH Ready flag after low power mode
  *     @arg FLASH_FLAG_ENDHV: FLASH End of high voltage flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *     @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *     @arg FLASH_FLAG_SIZERR: FLASH size error flag
  *     @arg FLASH_FLAG_OPTVERR: FLASH Option validity error flag
  *     @arg FLASH_FLAG_OPTVERRUSR: FLASH Option User validity error flag
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_FLASH_GET_FLAG(FLASH_FLAG));

  if((FLASH->SR & FLASH_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus; 
}

/**
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *     @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag 
  *     @arg FLASH_FLAG_SIZERR: FLASH size error flag    
  *     @arg FLASH_FLAG_OPTVERR: FLASH Option validity error flag
  *     @arg FLASH_FLAG_OPTVERRUSR: FLASH Option User validity error flag
  * @retval None
  */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: 
  *   FLASH_BUSY, FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status FLASHstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    FLASHstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    { 
      FLASHstatus = FLASH_ERROR_WRP;
    }
    else 
    {
      if((FLASH->SR & (uint32_t)0x1E00) != (uint32_t)0x00)
      {
        FLASHstatus = FLASH_ERROR_PROGRAM; 
      }
      else
      {
        FLASHstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the FLASH Status */
  return FLASHstatus;
}


/**
  * @brief  Waits for a FLASH operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH programming Timeout.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, 
  *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
  __IO FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the FLASH Status */
  status = FLASH_GetStatus();
  
  /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
  {
    status = FLASH_GetStatus();
    Timeout--;
  }
  
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

#endif /* STM32L1XX_MD */

/**
  * @}
  */

/**
  * @}
  */
   
  /**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
