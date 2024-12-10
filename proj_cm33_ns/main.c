/*****************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for non-secure
*                    application in the CM33 CPU
*
* Related Document : See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
 * Header Files
 *****************************************************************************/
 
#include "cybsp.h"
#include "cy_pdl.h"
#include "partition_psc3.h"
#include "cy_retarget_io.h"


/******************************************************************************
 * Macros
 *****************************************************************************/

/* Test data for memory read-write operations */
#define NON_SEC_TEST_DATA        0x76543210

/* Shared memory NS alias address */
#define SRAM0_NS_SAHB_SHM_ADDR   (SRAM0_NS_SAHB_START + SRAM0_SIZE - SRAM0_SHM_SIZE)

/* UART Message logging */
#if defined (ENABLE_NS_LOG) || (ENABLE_S_NS_LOG)
#define print_msg(...)           printf(__VA_ARGS__)
#else /* ENABLE_S_LOG */
#define print_msg(...)
#endif /* (ENABLE_NS_LOG) || (ENABLE_S_NS_LOG) */

/**
 * UART config for logging
 */
#if defined (ENABLE_NS_LOG)
#define DEBUG_UART_NS_HW                 DEBUG_UART_1_HW
#define DEBUG_UART_NS_config             DEBUG_UART_1_config
#define DEBUG_UART_NS_hal_config         DEBUG_UART_1_hal_config
#elif defined (ENABLE_S_NS_LOG)
#define DEBUG_UART_NS_HW                 DEBUG_UART_2_HW
#define DEBUG_UART_NS_config             DEBUG_UART_2_config
#define DEBUG_UART_NS_hal_config         DEBUG_UART_2_hal_config
#endif


/* Return response codes for test_memory() */
typedef enum {
    RESP_FAIL = -1,
    RESP_SUCCESS,
} memory_resp;

/* UART Message logging */
#if defined (ENABLE_NS_LOG) || (ENABLE_S_NS_LOG)

static cy_stc_scb_uart_context_t    DEBUG_UART_NS_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_NS_hal_obj;           /** Debug UART HAL object  */

#endif /* (ENABLE_NS_LOG) || (ENABLE_S_NS_LOG) */
/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * @brief Memory write-read test function.
 *
 * This function performs a write-read operation on a given address and returns
 * the status of the operation.
 * 
 * @param addr The pointer to memory.
 * @param data The data to be written on addr.
 * @return The status memory_resp.
 */
memory_resp test_memory(volatile uint32_t *addr, uint32_t data);

/**
 * @brief Read shred memory.
 *
 * This function performs a read operation on a given address and returns
 * the uint32_t value present at that address.
 * 
 * @param addr The pointer to memory.
 * @return The uint32_t value.
 */
uint32_t read_shared_mem(uint32_t *addr);


/******************************************************************************
 * Function Definations
 *****************************************************************************/

memory_resp test_memory(volatile uint32_t *addr, uint32_t data)
{
    uint32_t read_data = 0;

    *addr = data;
    read_data = *addr;
    if (read_data == data)
        return RESP_SUCCESS;
    return RESP_FAIL;
}

uint32_t read_shared_mem(uint32_t *addr)
{
    return (uint32_t)*addr;
}


/**
 * @brief Main function of M33 Non-secure project
 *
 * This is the non-secure main function for M33 CPU. It performs a series of
 * read/write operations on shared memory as well as access a Secure memory to
 * generate and handle fault.
 *
 * @return Should not return
 *
 */
int main(void)
{
    cy_rslt_t result = 0xFFFFFFFF;
    memory_resp response = RESP_FAIL;
    uint32_t read_data = 0;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#if defined (ENABLE_NS_LOG) || (ENABLE_S_NS_LOG)
    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_NS_HW, &DEBUG_UART_NS_config, &DEBUG_UART_NS_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_NS_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_NS_hal_obj, &DEBUG_UART_NS_hal_config, &DEBUG_UART_NS_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_NS_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif /* (ENABLE_NS_LOG || ENABLE_S_NS_LOG) */

    print_msg("\x1b[2J\x1b[;H");
    print_msg("****************** "
           "PSOC Control MCU: Memory Isolation"
           "****************** \r\n\n");
    print_msg("\r\nCM33 Non-secure App start...\r\n");

    /* Read the data passed by the secure application and print it. */
    print_msg("\r\nReading shared data passed by Secure app...\r\n");

    read_data = read_shared_mem((uint32_t*)SRAM0_NS_SAHB_SHM_ADDR);

    print_msg("Shared data passed by Secure app is 0x%08lx\r\n", (unsigned long)read_data);

    /**
     * Make Non-secure write/read transaction from PC5 to shared memory on SRAM1.
     * This access is allowed and should be successful.
     */
    response = test_memory((uint32_t*)SRAM0_NS_SAHB_SHM_ADDR, NON_SEC_TEST_DATA);
    if(RESP_SUCCESS == response)
    {
        print_msg("SUCCESS: Non-secure Read-Write operation on Non-secure memory"
               " from Secure application successful\r\n");
    }
    else
    {
        /* Unexpected behaviour, should not enter here */
        print_msg("FAIL: Non-secure Read-Write operation on Non-secure memory"
               " from Secure application failed\r\n");
        while(1);
    }

    /**
     * Access SRAM0 with NS alias. This should generate a SecureFault.
     * The fault handler configures a SAU region for SRAM0 as NS and
     * sets the MPC response to RZWI. The SRAM0 MPC is configured to
     * allow only S access for all PCs. Hence no S data is leaked or
     * modified by NS read/write operation.
     */

    print_msg("\r\nAccessing Secure memory with Non-secure address alias...\r\n");

    /* Should not return. It should enter SecureFault handler. */
    response = test_memory((uint32_t*)SRAM0_S_SAHB_START, NON_SEC_TEST_DATA);

    for (;;)
    {


    }
}

/* [] END OF FILE */
