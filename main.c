/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB CDC Echo Example
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*
*******************************************************************************
* (c) (2019), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_usb_dev.h"
#include "cycfg_usbdev.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define USBUART_BUFFER_SIZE (64u)
#define USBUART_COM_PORT    (0U)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void usb_high_isr(void);
static void usb_medium_isr(void);
static void usb_low_isr(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* USB Interrupt Configuration */
const cy_stc_sysint_t usb_high_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_hi_IRQn,
    .intrPriority = 5U,
};
const cy_stc_sysint_t usb_medium_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_med_IRQn,
    .intrPriority = 6U,
};
const cy_stc_sysint_t usb_low_interrupt_cfg =
{
    .intrSrc = (IRQn_Type) usb_interrupt_lo_IRQn,
    .intrPriority = 7U,
};

/* USBDEV context variables */
cy_stc_usbfs_dev_drv_context_t  usb_drvContext;
cy_stc_usb_dev_context_t        usb_devContext;
cy_stc_usb_dev_cdc_context_t    usb_cdcContext;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM4 CPU. It initializes the USB Device block
*  and enumerates as a CDC device. It constantly checks for data received from
*  host and echos it back. 
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t count;
    uint8_t buffer[USBUART_BUFFER_SIZE];

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the USB device */
    Cy_USB_Dev_Init(CYBSP_USBDEV_HW, &CYBSP_USBDEV_config, &usb_drvContext, 
                    &usb_devices[0], &usb_devConfig, &usb_devContext);

    /* Initialize the CDC Class */
    Cy_USB_Dev_CDC_Init(&usb_cdcConfig, &usb_cdcContext, &usb_devContext);

    /* Initialize the USB interrupts */
    Cy_SysInt_Init(&usb_high_interrupt_cfg,   &usb_high_isr);
    Cy_SysInt_Init(&usb_medium_interrupt_cfg, &usb_medium_isr);
    Cy_SysInt_Init(&usb_low_interrupt_cfg,    &usb_low_isr);   

    /* Enable the USB interrupts */
    NVIC_EnableIRQ(usb_high_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_medium_interrupt_cfg.intrSrc);
    NVIC_EnableIRQ(usb_low_interrupt_cfg.intrSrc);

    /* Make device appear on the bus. This function call is blocking, 
       it waits till the device enumerates */
    Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &usb_devContext);

    for(;;)
    {
        /* Check if host sent any data */
        if (Cy_USB_Dev_CDC_IsDataReady(USBUART_COM_PORT, &usb_cdcContext))
        {
            /* Get number of bytes */
            count = Cy_USB_Dev_CDC_GetAll(USBUART_COM_PORT, buffer, USBUART_BUFFER_SIZE, &usb_cdcContext);

            if (0u != count)
            {
                /* Wait until component is ready to send data to host */
                while (0u == Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext))
                {
                }

                /* Send data back to host */
                Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, buffer, count, &usb_cdcContext);

                /* If the last sent packet is exactly the maximum packet 
                *  size, it is followed by a zero-length packet to assure
                *  that the end of the segment is properly identified by 
                *  the terminal.
                */
                if (USBUART_BUFFER_SIZE == count)
                {
                    /* Wait until component is ready to send data to PC. */
                    while (0u == Cy_USB_Dev_CDC_IsReady(USBUART_COM_PORT, &usb_cdcContext))
                    {
                    }

                    /* Send zero-length packet to PC. */
                    Cy_USB_Dev_CDC_PutData(USBUART_COM_PORT, NULL, 0u, &usb_cdcContext);
                }
            }
        }

        /* Go to sleep */
        cyhal_system_sleep();
    }
}

/***************************************************************************
* Function Name: usb_high_isr
********************************************************************************
* Summary:
*  This function processes the high priority USB interrupts.
*
***************************************************************************/
static void usb_high_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseHi(CYBSP_USBDEV_HW), 
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_medium_isr
********************************************************************************
* Summary:
*  This function processes the medium priority USB interrupts.
*
***************************************************************************/
static void usb_medium_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(CYBSP_USBDEV_HW), 
                               &usb_drvContext);
}


/***************************************************************************
* Function Name: usb_low_isr
********************************************************************************
* Summary:
*  This function processes the low priority USB interrupts.
*
**************************************************************************/
static void usb_low_isr(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(CYBSP_USBDEV_HW, 
                               Cy_USBFS_Dev_Drv_GetInterruptCauseLo(CYBSP_USBDEV_HW), 
                               &usb_drvContext);
}


/* [] END OF FILE */