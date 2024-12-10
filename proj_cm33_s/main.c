/*****************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for secure
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
#include <arm_cmse.h>
#include "cy_pdl.h"
#include "partition_ARMCM33.h"
#include "cy_ppc.h"
#include "cy_ms_ctl.h"
#include "cy_retarget_io.h"


/******************************************************************************
 * Macros
 *****************************************************************************/

/* CM33 NS image stack pointer address */
#define CM33_SP_STORE                   NS_BOOT_START

/* CM33 NS image Reset handler address */
#define CM33_RESET_HANDLER_STORE        (NS_BOOT_START + 4)

/* Test data for memory read-write operations */
#define SEC_TEST_DATA                   0x01234567
#define SEC_SHM_DATA                    0x01020304

/* SRAM S-bus NS alias address for memory read-write operations */
#define SRAM0_NS_SAHB_NS_ADDR           (SRAM0_NS_SAHB_START + SRAM0_NS_OFFSET)

/* Shared memory secure alias address */
#define SRAM0_S_SAHB_SHM_ADDR           (SRAM0_S_SAHB_START + SRAM0_SIZE - SRAM0_SHM_SIZE)

/* UART Message logging */
#if defined(ENABLE_S_LOG) || (ENABLE_S_NS_LOG)
#define print_msg(...)                  printf(__VA_ARGS__)
#define DEBUG_UART_S_HW                 DEBUG_UART_1_HW
#define DEBUG_UART_S_config             DEBUG_UART_1_config
#define DEBUG_UART_S_hal_config         DEBUG_UART_1_hal_config
#else /* ENABLE_NS_LOG */
#define print_msg(...)
#endif /* defined(ENABLE_S_LOG) || (ENABLE_S_NS_LOG) */

/* Return response codes for test_memory() */
typedef enum {
    RESP_FAIL = -1,
    RESP_SUCCESS
} memory_resp;

/* Function pointer for NS function i.e. NonSecure_ResetHandler() */
typedef void (*funcptr_void) (void) __attribute__((cmse_nonsecure_call));


/*******************************************************************************
* Global Variables
*******************************************************************************/
/* UART Message logging */
#if defined(ENABLE_S_LOG) || (ENABLE_S_NS_LOG)

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_S_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_S_hal_obj;           /** Debug UART HAL object  */

#endif /* defined(ENABLE_S_LOG) || (ENABLE_S_NS_LOG) */

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * @brief Configure PPC.
 *
 * This function configures the peripheral protection controller (PPC) to partition
 * the peripherals in to secure and non-secure worlds. The resources which are unused
 * by secure project are configured as non-secure. 
 * 
 * @return The status cy_en_ppc_status_t.
 */
cy_en_ppc_status_t config_ppc(void);

/**
 * @brief Configure M33 Protection Context.
 *
 * This function configures the M33 PC. The active and saved PC is set to PC3
 * for non-secure application. 
 * 
 * @return The status cy_en_ms_ctl_status_t.
 */
cy_en_ms_ctl_status_t config_set_cm33_ns_pc(void);

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
 * @brief SecureFault handler function.
 * @return Void.
 */
void SecureFault_Handler(void);

/**
 * @brief Fault handler function.
 * @return Void.
 */
void FaultHandler(uint32_t const *faultStackAddr);


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

memory_resp test_memory(volatile uint32_t *addr, uint32_t data)
{
    uint32_t read_data = 0;

    *addr = data;
    read_data = *addr;
    if (read_data == data)
        return RESP_SUCCESS;
    return RESP_FAIL;
}

void SecureFault_Handler(void)
{
    __asm (
        "MRS R0, CONTROL\n"
        "TST R0, #2\n"
        "ITE EQ\n"
        "MRSEQ R0, MSP_NS\n"
        "MRSNE R0, PSP_NS\n"
        "B FaultHandler\n"
    );
}

void FaultHandler(uint32_t const *faultStackAddr)
{
    Cy_SysLib_FaultHandler(faultStackAddr);
}

/**
 * @brief Fault processing function function.
 *
 * This is the fault handler function. A weak implementation is provided by
 * the PDL, a strong implementation in provided here in application code.
 * This SecureFault handler should run in PC2. The SecureFault_Handler() has to
 * be registered as PC2 handler in the OEM policy.
 * 
 * @return Void.
 * 
 * @note This fault handler calls prinf() function to dump the exception stack
 * frame. In an actual use case, an exception/interrupt handler should avoid
 * calling such heavy functions as this affects the interrupt execution time.
 */
void Cy_SysLib_ProcessingFault(void)
{
    uint32_t SHSCR_reg = 0;

    SHSCR_reg = SCB->SHCSR;
    if(SHSCR_reg & SCB_SHCSR_SECUREFAULTACT_Msk)
    {
        print_msg("\r\nCM33 SecureFault!!\r\n");

        /* Print Non-secure Fault Frame */
        print_msg("Printing Non-secure exception stack frame...\r\n");

        print_msg("r0 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r0);
        print_msg("r1 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r1);
        print_msg("r2 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r2);
        print_msg("r3 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r3);
        print_msg("r12 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r12);
        print_msg("lr = 0x%08lx\r\n", (unsigned long)cy_faultFrame.lr);
        print_msg("pc = 0x%08lx\r\n", (unsigned long)cy_faultFrame.pc);
        print_msg("psr = 0x%08lx\r\n", (unsigned long)cy_faultFrame.psr);
    }

    while(true);
}

cy_en_ppc_status_t config_ppc(void)
{
    PPC_Type* ppcPtr = NULL;
    cy_en_ppc_status_t ppcStatus = CY_PPC_FAILURE;

    cy_stc_ppc_init_t ppcInit;
    cy_stc_ppc_attribute_t ppcAttribute;
    cy_stc_ppc_pc_mask_t pcMaskConfig;

    /* Configure PPC0 for CM33 access */
    ppcPtr = PPC;

    /* Initialize PPC */
    ppcInit.respConfig = CY_PPC_BUS_ERR;
    ppcStatus = Cy_Ppc_InitPpc(ppcPtr, &ppcInit);
    if (ppcStatus != CY_PPC_SUCCESS)
        return ppcStatus;

    ppcAttribute.secAttribute = CY_PPC_NON_SECURE;
    ppcAttribute.secPrivAttribute = CY_PPC_SEC_NONPRIV;
    ppcAttribute.nsPrivAttribute = CY_PPC_NON_SEC_NONPRIV;

    /* Address: 0x42000000 - 0x42004157 */
    ppcAttribute.startRegion = PROT_PERI0_MAIN;
    ppcAttribute.endRegion = PROT_PERI0_GR5_GROUP;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x42040000 - 0x4204FFFF */
    ppcAttribute.startRegion = PROT_PERI_PCLK0_MAIN;
    ppcAttribute.endRegion = PROT_PERI_PCLK0_MAIN;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x42100000 - 0x421102FF */
    ppcAttribute.startRegion = PROT_CPUSS;
    ppcAttribute.endRegion = PROT_RAMC0_RAM_PWR;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x42160000 - 0x421600FF */
    ppcAttribute.startRegion = PROT_MXCM33_CM33;
    ppcAttribute.endRegion = PROT_MXCM33_CM33;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x42168000 - 0x421c2007 */
    ppcAttribute.startRegion = PROT_MXCM33_CM33_INT;
    ppcAttribute.endRegion = PROT_CPUSS_BOOT;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x421c5004 - 0x421c51f4 */
    ppcAttribute.startRegion = PROT_MS_PC0_PRIV_MIR;
    ppcAttribute.endRegion = PROT_MS_PC31_PRIV_MIR;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x421c8000 - 0x42BFFFFF */
    ppcAttribute.startRegion = PROT_CPUSS_SL_CTL_GROUP;
    ppcAttribute.endRegion = PROT_MCPASS;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    /* Address: 0x42008000 - 0x4200FFFF */
    ppcAttribute.startRegion = PROT_PERI0_TR;
    ppcAttribute.endRegion = PROT_PERI0_TR;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

#if defined(ENABLE_S_LOG) || (ENABLE_S_NS_LOG)
    
    ppcAttribute.secAttribute = CY_PPC_SECURE;

    /* Address: 0x42850000 - 0x4285FFFF */
    ppcAttribute.startRegion = PROT_SCB3;
    ppcAttribute.endRegion = PROT_SCB3;
    ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }
#endif

    /* Set PC Mask */
    pcMaskConfig.startRegion = PROT_PERI0_MAIN;
    pcMaskConfig.endRegion = PROT_MCPASS;
    pcMaskConfig.pcMask = 0xFFFFFFFF;
    ppcStatus = Cy_Ppc_SetPcMask(ppcPtr, &pcMaskConfig);
    if (ppcStatus != CY_PPC_SUCCESS)
    {
        return ppcStatus;
    }

    return CY_PPC_SUCCESS;
}

cy_en_ms_ctl_status_t config_set_cm33_ns_pc(void)
{
    cy_en_ms_ctl_status_t mscStatus = CY_MS_CTL_FAILURE;

    mscStatus = Cy_Ms_Ctl_SetSavedPC(CPUSS_MS_ID_CM33_0, CM33_NS_PC_VALUE);
    if (mscStatus == CY_MS_CTL_SUCCESS)
    {
        mscStatus = Cy_Ms_Ctl_SetActivePC(CPUSS_MS_ID_CM33_0, CM33_NS_PC_VALUE);
    }
    return mscStatus;
}

/**
 * @brief Main function of M33 Secure project
 *
 * This is the main function for CPU. It
 *    1. Configures protection units
 *    2. Performs a series of read-write operation on memory to demonstrate
 *       isolation
 *    3. Launches non-secure application
 *
 * @return Should not return
 *
 */
int main(void)
{
    uint32_t ns_stack = 0;
    funcptr_void NonSecure_ResetHandler = NULL;
    cy_rslt_t result = 0xFFFFFFFF;
    cy_en_ms_ctl_status_t ms_ctl_status = CY_MS_CTL_FAILURE;
    cy_en_ppc_status_t ppc_status = CY_PPC_FAILURE;
    memory_resp response = RESP_FAIL;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
       CY_ASSERT(0);
    }

    /* Enable SecureFault */
    SCB->SHCSR |= SCB_SHCSR_SECUREFAULTENA_Msk;

    /* Register a fault handler */
    NVIC_SetVector(SecureFault_IRQn, (uint32_t) SecureFault_Handler);

    /* Enable global interrupts */
    __enable_irq();

#if (ENABLE_S_LOG || ENABLE_S_NS_LOG)
    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_S_HW, &DEBUG_UART_S_config, &DEBUG_UART_S_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_S_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_S_hal_obj, &DEBUG_UART_S_hal_config, &DEBUG_UART_S_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_S_hal_obj);

    /* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif /* (ENABLE_S_LOG || ENABLE_S_NS_LOG) */

    print_msg("\x1b[2J\x1b[;H");
    print_msg("****************** "
           "PSOC Control MCU: Memory Isolation"
           "****************** \r\n\n");
    print_msg("CM33 Secure App start...\r\n\n");


    /**
     * By default all the memories in the MCU are Secure. To launch the
     * Non-secure application and create an isolation boundary between them,
     * SAU and MPC needs to be configured correctly. In PSOC Control MCU the
     * MPC is configured by ROM. The ROM applies the MPC configuration
     * provided in the OEM policy during provisioning.
     * The memory map of the app is shown below:
     *
     *    SRAM 64KB          FLASH 256KB
     *   _____________      _____________
     *  |             |    |             |
     *  |  S (16KB)   |    | S (65280 B) |
     *  |_____________|    |_____________|
     *  |             |    |             |
     *  |             |    | NSC (256 B) |
     *  |  NS (44KB)  |    |_____________|
     *  |             |    |             |
     *  |_____________|    |             |
     *  |             |    |  NS (192KB) |
     *  |  SHM (4KB)  |    |             |
     *  |_____________|    |_____________|
     * 
     */

    /* Configure SAU for SRAM, Flash and Peri MMIO */
    TZ_SAU_Setup();

    /**
     * The MPC and SAU are have configured 42KB of SRAM as Non-secure.
     * A Non-secure transaction should be able to access this section. A NS
     * transaction can be initiated by using the NS alias of SRAM.
     */

    print_msg("Accessing Non-secure memory with Non-secure address alias...\r\n");
    response = test_memory((uint32_t*)SRAM0_NS_SAHB_NS_ADDR, SEC_TEST_DATA);

    if(RESP_SUCCESS == response)
    {
        print_msg("SUCCESS: Non-secure Read-Write operation on Non-secure memory "
                  "from Secure application successful\r\n");
    }
    else
    {
        /* Unexpected behaviour, should not enter here */
        print_msg("FAIL: Non-secure Read-Write operation on Non-secure memory "
                  "from Secure application failed\r\n");
        while(true);
    }


    /**
     * The last 4KB of SRAM0 is configured as shared memory. This section of
     * memory should be accessible to both secure as well as Non-secure
     * applications. From the secure application we make a secure read/write
     * transaction to shared memory and this should be successful.
     */

    print_msg("\r\nAccessing shared memory with Secure address alias...\r\n");
    response = test_memory((uint32_t*)SRAM0_S_SAHB_SHM_ADDR, SEC_SHM_DATA);
    if(RESP_SUCCESS == response)
    {
        print_msg("SUCCESS: Secure Read-Write operation from PC2 on shared "
                  "memory successful\r\n");
    }
    else
    {
        /* Unexpected behaviour, should not enter here */
        print_msg("FAIL: Secure Read-Write operation from PC2 on shared "
                  "memory failed\r\n");
        while(true);
    }

    /* Configure PPC */
    ppc_status = config_ppc();
    if (CY_PPC_SUCCESS != ppc_status)
    {
       CY_ASSERT(0);
    }

    ns_stack = (uint32_t)(*((uint32_t*)CM33_SP_STORE));
    __TZ_set_MSP_NS(ns_stack);

    NonSecure_ResetHandler = (funcptr_void)(*((uint32_t*)CM33_RESET_HANDLER_STORE));

    /* change pc value for cm33-ns */
    ms_ctl_status = config_set_cm33_ns_pc();
    if (CY_MS_CTL_SUCCESS != ms_ctl_status)
    {
       CY_ASSERT(0);
    }

    /* Start non-secure application */
    NonSecure_ResetHandler();

    /* Program should not reach here */
    for (;;)
    {

    }
}
/* [] END OF FILE */
