/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY      (7u)
#define EZI2C_INTR_PRIORITY         (6u) /* EZI2C interrupt priority must be
                                          * higher than CapSense interrupt */


/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;
uint8_t uart_read_value;
cyhal_gpio_t GPIO_PIN;
volatile bool capsense_scan_complete = false;
cy_rslt_t   rslt;
cyhal_pwm_t pwm_obj;
uint16_t duty_cycle=20;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/

static uint32_t initialize_capsense(void);
static void initialize_capsense_tuner(void);
static void process_touch(void);
static void capsense_callback();
static void capsense_isr(void);
void handle_error(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
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

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }


    /* Initialize retarget-io to use the debug UART port */
        result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CY_RETARGET_IO_BAUDRATE);
        /* Board init failed. Stop program execution */
           if (result != CY_RSLT_SUCCESS)
           {
               CY_ASSERT(0);
           }
           /* Enable global interrupts */
            __enable_irq();

            initialize_capsense_tuner();

            /*Intialize capsense*/
            result = initialize_capsense();
              if (CYRET_SUCCESS != result)
              {
                  /* Halt the CPU if CapSense initialization failed */
                  CY_ASSERT(0);
              }

              /* Initiate first scan */
              Cy_CapSense_ScanAllWidgets(&cy_capsense_context);


        while(1){
        int flag=0;
        int flag_2=0;
        while(flag==0){

        	 printf("Choose from below for GPIO pin:\n");
        	            printf("\r\n2:P5_2 \r\n");
        	            printf("\r\n7:P5_7\r\n");

           /* Check if key was pressed */
          if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 5000)
                        == CY_RSLT_SUCCESS){


        	   switch(uart_read_value)
        	   {


        	   	   case '2': GPIO_PIN=P5_2;
        	   		   	   	   printf("\r\n Hey..see the pwm at gpio pin P5.2!!!\r\n");

        	   		   	   	   flag=1;
        	   	   	   	   break;

        	   	   case '7': GPIO_PIN=P5_7;
        	   		   	 printf("\r\n Hey..see the pwm at gpio pin P5.7!!!\r\n");

        	   		   	 	 	 flag=1;
        	   	   	   	   break;
        	   	   default: break;

        	   }
           }
          // Initialize PWM on the supplied pin and assign a new clock
          	  rslt = cyhal_pwm_init(&pwm_obj, GPIO_PIN, NULL);

         // Set a duty cycle of 20% and frequency of 200Hz
          	  rslt = cyhal_pwm_set_duty_cycle(&pwm_obj, duty_cycle, 200);

          // Start the PWM output
           rslt = cyhal_pwm_start(&pwm_obj);



         while(flag_2 ==0)
         {
        	 if (capsense_scan_complete)
        	                  {
        	                      /* Process all widgets */
        	                      Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

        	                      /* Process touch input */
        	                      process_touch();

        	                      /* Establishes synchronized operation between the CapSense
        	                       * middleware and the CapSense Tuner tool.
        	                       */
        	                      Cy_CapSense_RunTuner(&cy_capsense_context);

        	                      /* Initiate next scan */
        	                      Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

        	                      capsense_scan_complete = false;
        	                  }
        	 if(cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1)
        	                      == CY_RSLT_SUCCESS){
        	          		 flag_2=1;
        	          		cyhal_pwm_stop	(&pwm_obj);
        	          		cyhal_pwm_free(&pwm_obj);

        	          	 }

        	}


         }
        }



}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;


    static uint32_t button0_status_prev;
    static uint32_t button1_status_prev;
    static uint16_t slider_pos_prev;


    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON1_WDGT_ID,
        CY_CAPSENSE_BUTTON1_SNS0_ID,
        &cy_capsense_context);

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect new touch on Button0 */
    if ((0u != button0_status) &&
        (0u == button0_status_prev))
    {
        //led_data.state = LED_ON;
       // led_update_req = true;
    	// Set a duty cycle of 30% and frequency of 200Hz
    	 rslt = cyhal_pwm_set_duty_cycle(&pwm_obj, 30, 200);
    	 // Start the PWM output
    	 rslt = cyhal_pwm_start(&pwm_obj);
	   	 printf("\r\n button0 Pressed and dutyCycle=30%!!!!\r\n");

    }

    /* Detect new touch on Button1 */
    if ((0u != button1_status) &&
        (0u == button1_status_prev))
    {
    	// Set a duty cycle of 70% and frequency of 200Hz
    	rslt = cyhal_pwm_set_duty_cycle(&pwm_obj, 70, 200);
    	// Start the PWM output
    	rslt = cyhal_pwm_start(&pwm_obj);
    	printf("\r\n button1 Pressed and dutyCycle=70%!!!!\r\n");
    }

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) &&
        (slider_pos != slider_pos_prev))
    {
    	duty_cycle = (slider_pos +20);
       if(duty_cycle >=70){
    	   duty_cycle=70;

       }
        rslt = cyhal_pwm_set_duty_cycle(&pwm_obj, duty_cycle, 200);
        // Start the PWM output
        rslt = cyhal_pwm_start(&pwm_obj);
        printf("\r\n slider dutyCycle %d \r\n",duty_cycle);
    }



    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_prev = slider_pos;
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration parameters */
    static const cy_stc_sysint_t capSense_intr_config =
    {
        .intrSrc = csd_interrupt_IRQn,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    cyhal_system_set_isr(csd_interrupt_IRQn, csd_interrupt_IRQn, CAPSENSE_INTR_PRIORITY, &capsense_isr);
    NVIC_ClearPendingIRQ(capSense_intr_config.intrSrc);
    NVIC_EnableIRQ(capSense_intr_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
            capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_rslt_t result;

    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;

    result = cyhal_ezi2c_init(&sEzI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

}



/* [] END OF FILE */
