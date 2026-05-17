/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved


    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.


    This file is part of the FreeRTOS distribution.


    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.


    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************


    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?


    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.


    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.


    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.


    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.


    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.


    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.


    1 tab == 4 spaces!
*/


/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.


This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.


This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.


Here is a description of the project's functionality:


The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.


The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html


The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html


An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html


The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.


The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.


The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().


The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/


/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"






/*-----------------------------------------------------------*/
//traffic light defines
#define red     0
#define yellow  1
#define green   2


static void prvSetupHardware( void );


//helper function which performs an ADC conversion and returns the result via pointer to int
void myADC_Convert(int *adc_val);


//task functions
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );


//software timer callback
static void Traffic_Light_Timer_Callback (TimerHandle_t xTimer);


//software timer used to control light state transitions
TimerHandle_t light_timer = NULL;


//queue handles
xQueueHandle timer_done_handle = 0;
xQueueHandle traffic_light_state_handle = 0;
xQueueHandle traffic_flow_handle_generator = 0;
xQueueHandle incoming_car_handle = 0;
xQueueHandle traffic_flow_handle_lightstate = 0;
/*-----------------------------------------------------------*/


//Function: enableClock, which enables clocks for GPIOC and ADC1
void enableClock() {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}
//Function: myGPIO_Init(), which configures the appropriate GPIO pins
void myGPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    //select multiple pins: GPIO Pins 0, 1, 2 are used for the traffic light LEDs,
    //while GPIO Pins 6, 7, 8 are used for the car LEDs
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    //configure these pins as output
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;


    //apply configuration
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}


//Function: myGPIO_Pot_Init, which configures GPIOC pin 3 as analog input for potentiometer
void myGPIO_Pot_Init() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    //set to analog mode
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    //ensure no pull-up/pull-down to get more accurate results
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //apply configuration
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}
//Function: myADC_Init, which initializes ADC1 for conversions
void myADC_Init() {
        ADC_InitTypeDef ADC_InitStruct;
        ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
        //setting ADC to be 12-bit and right-aligned means the value we
        //should get from the potentiometer should be from 0 to (2^12)-1 (4095)
        ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStruct.ADC_ScanConvMode = DISABLE;
        ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
        ADC_InitStruct.ADC_ExternalTrigConvEdge = DISABLE;
        //apply configuration and enable
        ADC_Init(ADC1, &ADC_InitStruct);
        ADC_Cmd(ADC1, ENABLE);
        //select channel 13
        ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
}




int main(void)
{
    //enable clocks
    enableClock();
    prvSetupHardware();
    //initialize GPIO and ADC
    myGPIO_Init();
    myGPIO_Pot_Init();
    myADC_Init();


    //QUEUES:
    /* Create the queue used by the queue send and queue receive tasks and add to the registry, for the benefit of kernel aware debugging.
    http://www.freertos.org/a00116.html */


    //timer expiry queue
    timer_done_handle = xQueueCreate(1, sizeof(int));
    //queue to hold the current colour of the traffic light
    traffic_light_state_handle = xQueueCreate (100, sizeof(int));
    //queue to hold current traffic flow value to be used in the traffic generation task
    traffic_flow_handle_generator = xQueueCreate (100, sizeof(int));
    //queue to hold values 0 or 1 (1 = incoming car, 0 = no incoming car)
    incoming_car_handle = xQueueCreate (100, sizeof(int));
    //queue to hold current traffic flow value to be used in the light state task
    traffic_flow_handle_lightstate = xQueueCreate (1, sizeof(int));


    //register queues for debugging
    vQueueAddToRegistry (traffic_light_state_handle, "Traffic Light State");
    vQueueAddToRegistry (traffic_flow_handle_generator, "Traffic Flow for Traffic Generator");
    vQueueAddToRegistry (incoming_car_handle, "incoming Car");
    vQueueAddToRegistry (timer_done_handle, "timer done");
    vQueueAddToRegistry (traffic_flow_handle_lightstate, "Traffic Flow for Light State");


    //TASKS:
    xTaskCreate (Traffic_Flow_Adjustment_Task, "Traffic Flow Adjustment Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate (Traffic_Generator_Task, "Traffic Generator Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate (Traffic_Light_State_Task, "Traffic Light State Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate (System_Display_Task, "System Display Task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);


    //TIMER:
    //we initialize it as one-shot timer with 1000ms duration, but we will change that based on potentiometer/ADC value
    light_timer = xTimerCreate("Timer", pdMS_TO_TICKS(1000), pdFALSE, NULL, Traffic_Light_Timer_Callback);


    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    return 0;
}


//Helper Function: myADC_Convert, which performs ADC conversion and stores result in
//int that is passed into function as parameter of type int*
void myADC_Convert(int *adc_val) {
        //start conversion
        ADC_SoftwareStartConv(ADC1);
        //wait until end-of-conversion flag is set
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
        //update result
        *adc_val = ADC_GetConversionValue(ADC1);
}


//Timer Callback Function: when timer expires, this function is automatically executed
static void Traffic_Light_Timer_Callback (TimerHandle_t xTimer) {
    //the value we send to timer_done_handle doesn't matter;
    //we send an arbitrary value to fill the queue (which has max. capacity = 1), and
    //this value is sent to the Traffic_Light_State_Task to signal that the timer is done and
    //it's now time to change the duration of the timer (based on POT input) and start it again.
    int time_to_switch_colour = 1;
    xQueueSend(timer_done_handle, &time_to_switch_colour, 500);
}




/*-----------------------------------------------------------*/


static void Traffic_Flow_Adjustment_Task( void *pvParameters )
{
    int adc;
    while(1) {
            //read POT value and store result in int
            myADC_Convert(&adc);


            //convert ADC value (0-4095) to percentage
            int percent = (int)(adc * 100) / 4095;
            //add 20 to percent so that at minimum POT setting, there are still
            //some cars coming in
            percent += 20;


            //make sure it doesn't go over 100 (since we added 20)
            if (percent > 100) {
                percent = 100;
            }


            //send traffic percentage to traffic_flow_handle_generator and
            //traffic_flow_handle_lightstate
            xQueueSend(traffic_flow_handle_generator, &percent, 500);
            xQueueOverwrite(traffic_flow_handle_lightstate, &percent);
            vTaskDelay(500);
    }
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
static void Traffic_Generator_Task( void *pvParameters )
{
    int percent;
    while(1) {
        //as default, assume no car is entering the intersection
        int incoming_car = 0;
        //receive current traffic flow percentage (from 20 to 100)
        if (xQueueReceive(traffic_flow_handle_generator, &percent, 500)) {
            //use rand() function to generate random number and check if
            //that number is less than our flow percentage.
            //so if the percentage is high, the probability of getting a random
            //number that satisfies the condition is also high, which corresponds
            //to incoming_car being set to 1
            if ((rand() % 100) < percent ) {
                incoming_car = 1;
            }
        }
        //send incoming_car value to incoming_car_handle queue
        xQueueSend(incoming_car_handle, &incoming_car, 500);
        vTaskDelay(500);
    }
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


static void Traffic_Light_State_Task( void *pvParameters )
{
    int colour = green;
    int arbitrary_value;
    int percent;


    int start = green;


    //this is where we initially start the timer
    xQueueSend(traffic_light_state_handle, &start, 500);
    xTimerChangePeriod(light_timer, pdMS_TO_TICKS(2000), 0);
    xTimerStart(light_timer, 0);


    //set max durations (in ms) for each light colour in the traffic light
    int green_max_duration = 12000;
    int yellow_duration = 3000; //will always stay constant
    int red_max_duration = 6000;


    int total_cycle_duration_max = green_max_duration + red_max_duration;


    while(1)
    {   //wait until we can consume from the timer_done_handle queue (which signals that
        //the timer has expired and we need to start it again)
        if(xQueueReceive(timer_done_handle, &arbitrary_value, 500)) {


            //once we reach here, timer_done_handle should be empty until next timer callback
            xQueueReceive(traffic_flow_handle_lightstate, &percent, 500);


            //compute current durations based on flow percent
            int new_percent = (percent - 20) * 100 /(100-20);
            int green_cur_duration = (total_cycle_duration_max/3) + (new_percent * total_cycle_duration_max)/300;
            int red_cur_duration =  total_cycle_duration_max - green_cur_duration;


            if(colour == red)
            {
                //using computed red duration, we change the period of the timer and start it again
                xQueueSend(traffic_light_state_handle, &colour, 500);
                xTimerChangePeriod(light_timer, pdMS_TO_TICKS(red_cur_duration), 0);
                xTimerStart(light_timer, 0);


            }
            else if (colour == yellow) {
                //using yellow duration, we change the period of the timer and start it again
                xQueueSend(traffic_light_state_handle, &colour, 500);
                xTimerChangePeriod(light_timer, pdMS_TO_TICKS(yellow_duration), 0);
                xTimerStart(light_timer, 0);
            } else {
                //using computed green duration, we change the period of the timer and start it again
                xQueueSend(traffic_light_state_handle, &colour, 500);
                xTimerChangePeriod(light_timer, pdMS_TO_TICKS(green_cur_duration), 0);
                xTimerStart(light_timer, 0);
            }
            //after starting timer for current light colour, we advance to the next colour
            //so once the timer finishes, we turn on the next light in a sequence
            if (colour == red) {
                colour = green;
            } else if (colour == green) {
                colour = yellow;
            } else {
                colour = red;
            }
        }
        vTaskDelay(500);
    }
}


/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


static void System_Display_Task( void *pvParameters )
{
    int incoming_car;
    //represent car LEDs as array where 0 = no car (led off) and 1 = car present (led on)
    int cars[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int colour;


    while(1)
    {
        //get current light colour
        xQueueReceive(traffic_light_state_handle, &colour, 0);


        //control traffic light LEDs
        if (colour == red) {
            GPIO_SetBits(GPIOC, GPIO_Pin_0);
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        } else if (colour == yellow) {
            GPIO_SetBits(GPIOC, GPIO_Pin_1);
            GPIO_ResetBits(GPIOC, GPIO_Pin_0);
            GPIO_ResetBits(GPIOC, GPIO_Pin_2);
        } else {
            GPIO_SetBits(GPIOC, GPIO_Pin_2);
            GPIO_ResetBits(GPIOC, GPIO_Pin_0);
            GPIO_ResetBits(GPIOC, GPIO_Pin_1);
        }


        //on incoming car event, we enter this if-statement
        if (xQueueReceive(incoming_car_handle, &incoming_car, 500)) {


            //display car position
            for (int idx = 18; idx >= 0; idx--) {
                if (cars[idx] == 1) {
                    GPIO_SetBits(GPIOC, GPIO_Pin_6);
                } else {
                    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
                }
                //clock drives shifting
                GPIO_ResetBits(GPIOC,GPIO_Pin_7);
                GPIO_SetBits(GPIOC,GPIO_Pin_7);
            }


            xQueuePeek(traffic_light_state_handle, &colour, 0);
            //move cars depending on light state
            if (colour == green) {
                //if green, we simply shift all elements of array from position
                //idx-1 to idx, since we are iterating through the array backwards
                for (int idx = 18; idx > 0; idx--) {
                    cars[idx] = cars[idx-1];
                    //clear old position after moving car, since it's not there anymore
                    cars[idx-1] = 0;
                }
            } else {
                //if red or yellow, we move cars normally after the stop line, but
                //compress cars before the stop line


                for(int idx = 18; idx > 8; idx--){
                    //indices 8 to 18 represent part of road past the stop line
                    //in this section of the road, cars can move forward regardless of light state
                    cars[idx] = cars[idx-1];
                    cars[idx-1] = 0;
                }


                for(int idx = 7; idx > 0; idx--){
                    //indices 0 to 7 represent part of the road before the stop line
                    //in this section, cars only move forward if there is space
                    if(cars[idx] == 0){
                        //if the current position is empty, we allow a car to move forward
                        cars[idx] = cars[idx-1];
                        //clear old position after moving car, since it's not there anymore
                        cars[idx-1] = 0;
                    }
                    //if the current position is not empty, we don't allow a car to move forward from behind
                }
            }


            if (incoming_car == 1) {
                //if new car arrives, place it a beginning of road (index 0 of array)
                cars[0] = 1;
            } else {
                //otherwise, do nothing
            }


        }
        //reset shift register
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        vTaskDelay(500);
    }
}


/*-----------------------------------------------------------*/


void vApplicationMallocFailedHook( void )
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.


    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    for( ;; );
}
/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;


    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for( ;; );
}
/*-----------------------------------------------------------*/


void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;


    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.


    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    xFreeStackSpace = xPortGetFreeHeapSize();


    if( xFreeStackSpace > 100 )
    {
        /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
    }
}
/*-----------------------------------------------------------*/


static void prvSetupHardware( void )
{
    /* Ensure all priority bits are assigned as preemption priority bits.
    http://www.freertos.org/RTOS-Cortex-M3-M4.html */
    NVIC_SetPriorityGrouping( 0 );
    /* TODO: Setup the clocks, etc. here, if they were not configured before
    main() was called. */
}
