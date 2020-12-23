/*!
    \file  main.c
    \brief running led
    
    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>
#include <ff.h>
#include "diskio.h"


/* BUILTIN LED RED COLOR OF LONGAN BOARDS IS PIN PC13 */
// #define LED_PIN GPIO_PIN_13
// #define LED_GPIO_PORT GPIOC
// #define LED_GPIO_CLK RCU_GPIOC

/* BUILTIN LED GREEN*/
#define LED_PIN BIT(1)
#define LED_GPIO_PORT GPIOA
#define LED_GPIO_CLK RCU_GPIOA


uint8_t spi0_send_array[256] = {};
uint8_t spi0_receive_array[256]; 


void longan_led_on()
{
    GPIO_BC(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_off()
{
    GPIO_BOP(LED_GPIO_PORT) = LED_PIN;
}

void longan_led_init()
{
    /* enable the led clock */
    rcu_periph_clock_enable(LED_GPIO_CLK);
    /* configure led GPIO port */ 
    gpio_init(LED_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);

    longan_led_off();
}


void longan_bp1_init()
{
    /* enable the KEY_B clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure button pin as input */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
}

static void init_uart0(void)
{
   // enable GPIO clock 
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

   // enable USART0 clock 
   rcu_periph_clock_enable(RCU_USART0);  
   // configure USART0
   usart_deinit(USART0);
   usart_baudrate_set(USART0, 115200U);
   usart_word_length_set(USART0, USART_WL_8BIT);
   usart_stop_bit_set(USART0, USART_STB_1BIT);
   usart_parity_config(USART0, USART_PM_NONE);
   usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
   usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
   usart_receive_config(USART0, USART_RECEIVE_ENABLE);
   usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
   usart_enable(USART0);
}

// retarget the C library printf function to USART0
int _put_char(int ch) // used by printf
{
     usart_data_transmit(USART0, (uint8_t) ch );
     while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET){
     }
     return ch;
}

FIL File[2];		/* File object */
DIR Dir;			/* Directory object */
FILINFO Finfo;

static
FRESULT scan_files (
	char* path		/* Pointer to the working buffer with start path */
)
{
	DIR dirs;
	FRESULT fr;
	int i;

	fr = f_opendir(&dirs, path);
	if (fr == FR_OK) {
		while (((fr = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (Finfo.fattrib & AM_DIR) {
			
				i = strlen(path);
                printf("/%s\r\n", Finfo.fname);
				//if (fr != FR_OK) break;
			} else {
//				xprintf(PSTR("%s/%s\n"), path, Finfo.fname);
				printf("%s\r\n", Finfo.fname);
			}
		}
	}

	return fr;
}

void TIMER0_UP_IRQHandler(void)
{
    
    
    timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
}


void timer0_irq_init(void)
{
    timer_parameter_struct tpa;

    rcu_periph_clock_enable(RCU_TIMER0);
    timer_struct_para_init(&tpa);
    tpa.prescaler = 1080 - 1;       // prescaler (108MHz -> 100KHz)
    tpa.period = 100 - 1;           // max value of counting up (100KHz -> 1000Hz = 1ms)
    tpa.repetitioncounter = 1 - 1; // the num of overflows that issues update IRQ. (1ms*10 = 10ms)
    timer_init(TIMER0, &tpa);
    timer_interrupt_enable(TIMER0, TIMER_INT_UP);
    timer_enable(TIMER0);

    eclic_global_interrupt_enable();
    eclic_enable_interrupt(TIMER0_UP_IRQn);
}

void timer0_irq_stop(void)
{
    timer_interrupt_disable(TIMER0, TIMER_INT_UP);
    timer_disable(TIMER0);
}


static volatile uint32_t msTicks = 0;
static volatile bool tick_1s = false;
static volatile uint32_t tick_1s_counter = 0;

#define CONFIG_TICKS        (TIMER_FREQ / 1000)
#define SysTick_Handler     eclic_mtip_handler

void SysTick_Handler(void)
{                                /* SysTick interrupt Handler. */
    SysTick_Reload(CONFIG_TICKS);                            /* Call SysTick_Reload to reload timer. */
    msTicks++;                                                /* See startup file startup_gd32vf103.S for SysTick vector */
    tick_1s_counter++;
    if (tick_1s_counter >= 1000)
    {
        tick_1s_counter = 0;
        tick_1s = true;
    }
    disk_timerproc();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    longan_led_init();
    init_uart0();
    longan_bp1_init();

    uint32_t returnCode = SysTick_Config(CONFIG_TICKS);

   // timer0_irq_init(); // for TIMER0_UP_IRQHandler

    uint32_t bp1_counter = 0;
    uint32_t led_counter = 0;

    bool bp1_is_on = false;
    bool led_is_on = false;
    bool led_running = false;

  // Mount FAT
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1); // 0: mount successful ; 1: mount failed

    if (fr) { // Mount Fail (Loop)
        printf("[OST] No Card Found!\r\n", 1);
    }

    printf("[OST] SD Card File System = %d\r\n", fs.fs_type); // FS_EXFAT = 4
    printf("[OST] Starting with CPU=%d\r\n", (int)SystemCoreClock);

/*
    acc_size = acc_dirs = acc_files = 0;
				strcpy((char*)Buff, ptr);
				fr = scan_files((char*)Buff, &acc_dirs, &acc_files, &acc_size);
*/
    while(1)
    {
        if (tick_1s)
        {
            tick_1s = false;
            printf("[OST] SysTick=%d\r\n", (int)msTicks);

            if(SET ==  gpio_input_bit_get(GPIOB, GPIO_PIN_11))
            {
                if (!bp1_is_on)
                {
                    bp1_counter++;
                    if (bp1_counter >= 50)
                    {
                        bp1_counter = 0;
                        bp1_is_on = true;
                        printf("ON\n");
                    }
                }
            }
            else
            {
                if (bp1_is_on)
                {
                    bp1_counter++;
                    if (bp1_counter >= 50)
                    {
                        bp1_counter = 0;
                        bp1_is_on = false;
                    }
                }
            }
            
            if (bp1_is_on)
            {
                if (!led_running)
                {
                    led_running = true;
                    led_counter = 0;
                }
            }

            if (led_running)
            {
                led_counter++;
                if (led_counter >= 1000)
                {
                    led_counter = 0;
                    if (led_is_on)
                    {
                        printf("OFF\n");
                        longan_led_off();
                        led_is_on = false;
                        led_running = false;
                    }
                    else
                    {
                        led_is_on = true;
                        printf("ON\n");
                        longan_led_on();
                    }      
                }
            }
        }
        
    }
}
