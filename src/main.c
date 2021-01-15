#include "gd32vf103.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>
#include <ff.h>
#include "diskio.h"
#include "audio.h"
#include <string.h>

// filename of wave file to play
const char filename[] = "bensound-ukulele.wav"; //castemere_mono.wav";

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2
//
// Set DISABLE_CHIP_SELECT to disable a second SPI device.
// For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
// to 10 to disable the Ethernet controller.
const int8_t DISABLE_CHIP_SELECT = -1;

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

	fr = f_opendir(&dirs, path);
	if (fr == FR_OK)
    {
		while (((fr = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (Finfo.fattrib & AM_DIR) {
			
//				i = strlen(path);
                printf("/%s\r\n", Finfo.fname);
				if (fr != FR_OK) break;
			} else {
//				xprintf(PSTR("%s/%s\n"), path, Finfo.fname);
				printf("%s\r\n", Finfo.fname);
			}
		}
	}
    else
    {
        printf("[OST] Cannot open directory\r\n");
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

static volatile bool tick_10ms = false;
static volatile uint32_t tick_10ms_counter = 0;

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

    tick_10ms_counter++;
    if (tick_10ms_counter >= 10)
    {
        tick_10ms_counter = 0;
        tick_10ms = true;
    }

    disk_timerproc();
}
#if 0
 #include <math.h>
 #include "i2s.h"
#define SIZE_OF_SAMPLES 512  // samples for 2ch
#define SAMPLE_RATE     (44100)
#define WAVE_FREQ_HZ    (440)
#define PI              (3.14159265)
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)
#define DELTA 2.0*PI*WAVE_FREQ_HZ/SAMPLE_RATE

uint32_t audio_buf0[SIZE_OF_SAMPLES];
uint32_t audio_buf1[SIZE_OF_SAMPLES];

static double ang = 0;
static int count = 0;
static double triangle_float = 0.0;

union U {
    uint32_t i;
    uint16_t s[2];
} u;

// 16bit swap
uint32_t swap16b(uint32_t in_val)
{
    u.i = in_val;
    return ((uint32_t) u.s[0] << 16) | ((uint32_t) u.s[1]);
}

double _square_wave(void)
{
    double dval;
    if (ang >= 2.0*PI) {
        ang -= 2.0*PI;
        triangle_float = -(double) pow(2, 22);
    }
    if (ang < PI) {
        dval = 1.0;
    } else {
        dval = -1.0;
    }
    return dval;
}

// Generate Triangle Wave
void setup_triangle_sine_waves(int32_t *samples_data)
{
    unsigned int i;
    double square_float;
    double triangle_step = (double) pow(2, 23) / SAMPLE_PER_CYCLE;

    for(i = 0; i < SIZE_OF_SAMPLES/2; i++) {
        square_float = _square_wave();
        ang += DELTA;
        if (square_float >= 0) {
            triangle_float += triangle_step;
        } else {
            triangle_float -= triangle_step;
        }

        square_float *= (pow(2, 23) - 1);
        samples_data[i*2+0] = swap16b((int) square_float * 256);
        samples_data[i*2+1] = swap16b((int) triangle_float * 256);
    }
}

#define SAMPLERATE_HZ 44100  // The sample rate of the audio.  Higher sample rates have better fidelity,
                             // but these tones are so simple it won't make a difference.  44.1khz is
                             // standard CD quality sound.

#define AMPLITUDE     20   // Set the amplitude of generated waveforms.  This controls how loud
                             // the signals are, and can be any value from 0 to 2**31 - 1.  Start with
                             // a low value to prevent damaging speakers!
/*
#define SIZE_OF_SAMPLES      256    // The size of each generated waveform.  The larger the size the higher
                             // quality the signal.  A size of 256 is more than enough for these simple
                             // waveforms.

*/
// Define the frequency of music notes (from http://www.phy.mtu.edu/~suits/notefreqs.html):
#define C4_HZ      261.63
#define D4_HZ      293.66
#define E4_HZ      329.63
#define F4_HZ      349.23
#define G4_HZ      392.00
#define A4_HZ      440.00
#define B4_HZ      493.88

// Define a C-major scale to play all the notes up and down.
float scale[] = { C4_HZ, D4_HZ, E4_HZ, F4_HZ, G4_HZ, A4_HZ, B4_HZ, A4_HZ, G4_HZ, F4_HZ, E4_HZ, D4_HZ, C4_HZ };

// Store basic waveforms in memory.
int16_t sine[SIZE_OF_SAMPLES / 2]     = {0};


void generateSine(int32_t amplitude, int16_t* buffer, uint16_t length) {
  // Generate a sine wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  for (int i=0; i<length; ++i) {
    buffer[i] = (int16_t)(((float)amplitude)*sin(2.0*PI*(1.0/length)*i));
  }
}

void prepare_audio_buf(void)
{
//    generateSine(100, sine, SIZE_OF_SAMPLES / 2);
 //   setup_triangle_sine_waves(audio_buf0);
 //   setup_triangle_sine_waves(audio_buf1);
    // for(unsigned int i = 0; i < SIZE_OF_SAMPLES/2; i++)
    // {
    //     audio_buf0[i*2+0] = sine[i];
    //     audio_buf0[i*2+1] = sine[i];
    // }


    setup_triangle_sine_waves(audio_buf0);
    setup_triangle_sine_waves(audio_buf1);

    init_i2s2();
    init_dma_i2s2(audio_buf0, SIZE_OF_SAMPLES*2);

    spi_dma_enable(SPI2, SPI_DMA_TRANSMIT);
    dma_channel_enable(DMA1, DMA_CH1);
    count = 0;
}

void run_audio_buf(void)
{
    if (SET == dma_flag_get(DMA1, DMA_CH1, DMA_FLAG_FTF)) {
        dma_flag_clear(DMA1, DMA_CH1, DMA_FLAG_FTF);
        dma_channel_disable(DMA1, DMA_CH1);
        if (count % 2 == 0) {
            init_dma_i2s2(audio_buf1, SIZE_OF_SAMPLES*2);
        } else {
            init_dma_i2s2(audio_buf0, SIZE_OF_SAMPLES*2);
        }
        dma_channel_enable(DMA1, DMA_CH1);

        // 次のaudio_bufの準備をここで行う

        count++;
        
    }
}
#endif

static FATFS fs;

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

    (void) SysTick_Config(CONFIG_TICKS);

   // timer0_irq_init(); // for TIMER0_UP_IRQHandler

    uint32_t bp1_counter = 0;
    uint32_t led_counter = 0;

    bool bp1_is_on = false;
    bool led_is_on = false;
    bool led_running = false;

  // Mount FAT
    
    uint32_t retries = 3;
    FRESULT fr;
    do 
    {
        fr = f_mount(&fs, "", 1); // 0: mount successful ; 1: mount failed
        delay_1ms(10);
    } while (--retries && fr);

    if (fr)
    {
        printf("[OST] No Card Found! Err=%d\r\n", (int)fr);
    }

    printf("[OST] SD Card File System = %d\r\n", fs.fs_type); // FS_EXFAT = 4
    printf("[OST] Starting with CPU=%d\r\n", (int)SystemCoreClock);


    scan_files("");

    printf("[OST] Starting loop\r\n");

   audio_init();
    audio_play(filename);

  // prepare_audio_buf();

    while(1)
    {
    //    run_audio_buf();


        if (tick_1s)
        {
            tick_1s = false;
            //  printf("[OST] SysTick=%d\r\n", (int)msTicks);
        }


        if (tick_10ms)
        {
            tick_10ms = false;
          
            if(SET ==  gpio_input_bit_get(GPIOB, GPIO_PIN_11))
            {
                if (!bp1_is_on)
                {
                    bp1_counter++;
                    if (bp1_counter >= 5)
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
                    if (bp1_counter >= 5)
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
                if (led_counter >= 100)
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
