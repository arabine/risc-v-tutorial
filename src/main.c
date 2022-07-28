#include "gd32vf103.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>
#include <ff.h>
#include "diskio.h"
#include "audio.h"
#include <string.h>
#include "spi0.h"
#include "ili9341.h"

#include "libutil.h"

// filename of wave file to play
const char filename[] = "out.wav"; //castemere_mono.wav";

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

static uint8_t bmpImage[512];

static uint8_t decompressed[1024];
static uint8_t palette[16*4];

typedef struct {
   uint16_t type;                 /* Magic identifier            */
   uint32_t size;                       /* File size in bytes          */
   uint16_t reserved1;
   uint16_t reserved2;
   uint32_t offset;                     /* Offset to image data, bytes */
} bmp_header_t;

typedef struct {
   uint32_t size;               /* Header size in bytes      */
   uint32_t width;
   uint32_t height;                /* Width and height of image */
   uint16_t planes;       /* Number of colour planes   */
   uint16_t bits;         /* Bits per pixel            */
   uint32_t compression;        /* Compression type          */
   uint32_t imagesize;          /* Image size in bytes       */
   uint32_t xresolution;
   uint32_t yresolution;     /* Pixels per meter          */
   uint32_t ncolours;           /* Number of colours         */
   uint32_t importantcolours;   /* Important colours         */
   uint32_t rgb;
   uint32_t rgb2;
} bmp_infoheader_t;

static const uint32_t HEADER_SIZE = 14;
static const uint32_t INFO_HEADER_SIZE = 40;

uint8_t parse_bmp(const uint8_t *data, bmp_header_t *header, bmp_infoheader_t *info_header)
{
    uint8_t isBmp = 0;

    // Header is 14 bytes length
    isBmp = (data[0] == 'B') && (data[1] == 'M') ? 1 : 0;
    header->size = leu32_get(data + 2);
    header->offset = leu32_get(data + 10);

    info_header->size = leu32_get(data + HEADER_SIZE);
    info_header->width = leu32_get(data + HEADER_SIZE + 4);
    info_header->height = leu32_get(data + HEADER_SIZE + 8);
    info_header->planes = leu16_get(data + HEADER_SIZE + 12);
    info_header->bits = leu16_get(data + HEADER_SIZE + 14);
    info_header->compression = leu32_get(data + HEADER_SIZE + 16);
    info_header->imagesize = leu32_get(data + HEADER_SIZE + 20);
    info_header->xresolution = leu32_get(data + HEADER_SIZE + 24);
    info_header->yresolution = leu32_get(data + HEADER_SIZE + 28);
    info_header->ncolours = leu32_get(data + HEADER_SIZE + 32);
    info_header->importantcolours = leu32_get(data + HEADER_SIZE + 36);
    info_header->rgb = leu32_get(data + HEADER_SIZE + 40);
    info_header->rgb2 = leu32_get(data + HEADER_SIZE + 44);

    return isBmp;
}

void decompress()
{
    FIL fil;
    uint32_t offset;
    UINT br;
    FRESULT fr = f_open(&fil, "0_000_314CBAA1.bmp", FA_READ);
    if (fr != FR_OK)
    {
        printf("ERROR: f_open %d\n\r", (int) fr);
    }

    offset = 0;
    f_lseek(&fil, offset);
    f_read(&fil, bmpImage, 512, &br);

    int nblines=0;

    bmp_header_t header;
    bmp_infoheader_t info_header;
    parse_bmp(bmpImage, &header, &info_header);

    // Compute some sizes
    uint32_t fileSize = header.size;
    // uint32_t compressedSize = fileSize - header.offset;
    uint32_t paletteSize = header.offset - (HEADER_SIZE + INFO_HEADER_SIZE);

    // Copy palette
    offset = HEADER_SIZE + INFO_HEADER_SIZE;
    memcpy(palette, bmpImage + HEADER_SIZE + INFO_HEADER_SIZE, paletteSize);

    offset = header.offset;
    uint8_t *compressed = &bmpImage[header.offset];

/*
    printf("File size (from header):%d\r\n", (uint32_t)header.size);
    printf("File size (from data):%d\r\n", (uint32_t)fileSize);
    printf("Data offset:%d\r\n", (uint32_t)header.offset);
    printf("Image size:%d\r\n", (uint32_t)info_header.size);
    printf("width:%d\r\n", (uint32_t)info_header.width);
    printf("height:%d\r\n", (uint32_t)info_header.height);
    printf("Planes:%d\r\n", (uint32_t)info_header.planes);
    printf("Bits:%d\r\n", (uint32_t)info_header.bits);
    printf("Compression:%d\r\n", (uint32_t)info_header.compression); // 2 - 4 bit run length encoding
    printf("Image size:%d\r\n", (uint32_t)info_header.imagesize);
    printf("X resolution:%d\r\n", (uint32_t)info_header.xresolution);
    printf("Y resolution:%d\r\n", (uint32_t)info_header.yresolution);
    printf("Colors:%d\r\n", (uint32_t)info_header.ncolours);
    printf("Important colors:%d\r\n", (uint32_t)info_header.importantcolours);
    printf("RGB :%d\r\n", (uint32_t)info_header.rgb);
    printf("RGB2 :%d\r\n", (uint32_t)info_header.rgb2);
*/
    // buffer de sortie, bitmap décompressé
    memset(decompressed, 0, sizeof(decompressed));

  //  btea((uint32_t*) bmpImage, -128, key);

    uint32_t pixel = 0; // specify the pixel offset
    bool end = false;
    uint32_t i = 0;
    uint32_t totalPixels = 0;
    rect_t pos;
    pos.x = 0;
    pos.y = 0;
    pos.width = info_header.width;
    pos.height = 1;
    do
    {
        // if we are behond the middle of the buffer, read more data from file
        if (i > 256)
        {
            offset = offset + i;
            f_lseek(&fil, offset);
            f_read(&fil, bmpImage, 512, &br);
            i = 0;
        }
        
        uint8_t rleCmd = compressed[i];
        if (rleCmd > 0)
        {
            uint8_t val = compressed[i + 1];
            // repeat number of pixels
            for (uint32_t j = 0; j < rleCmd; j++)
            {
                if ((j & 1) == 0)
                {
                    decompressed[pixel] = (val & 0xF0) >>4;
                }
                else
                {
                    decompressed[pixel] = (val & 0x0F);
                }
                pixel++;

                if (pixel > info_header.width)
                {
                    // // enough pixels to write a line to the screen
                    // ili9341_draw_h_line(pos.y, decompressed, palette);
                    // // ili9341_write(&pos, decompressed);
                    // // next line...
                    // pos.y++;
                    // totalPixels += info_header.width;
                //    pixel = 0;
               //     nblines++;
               printf("!");
                }
            }
            i += 2; // jump pair instruction
        }
        else
        {
            uint8_t second = compressed[i + 1];
            if (second == 0)
            {
                if (pixel % info_header.width)
                {
                    // end of line
                    uint32_t lines = pixel / info_header.width;
                    uint32_t remaining = info_header.width - (pixel - (lines * info_header.width));

                    pixel += remaining;
                }
                i += 2;
            }
            else if (second == 1)
            {
                end = true;
            }
            else if (second == 2)
            {
                // delta N pixels and M lines
                pixel += compressed[i + 2] + compressed[i + 3] * info_header.width;
                i += 4;
            }
            else
            {
                // absolute mode
                uint8_t *ptr = &compressed[i + 2];
                // repeat number of pixels
                for (uint32_t j = 0; j < second; j++)
                {
                    if ((j & 1) == 0)
                    {
                        decompressed[pixel] = (*ptr & 0xF0) >> 4;
                    }
                    else
                    {
                        decompressed[pixel] = (*ptr & 0x0F);
                        ptr++;
                    }
                    pixel++;

                    if (pixel >= info_header.width)
                    {
                        printf("!");
                      //  pixel = 0;
                    }
                }
                i += 2 + (second / 2);

                // padded in word boundary, jump if necessary
                if ((second / 2) % 2)
                {
                    i++;
                }
            }

            if (pixel == info_header.width)
            {
                // enough pixels to write a line to the screen
                ili9341_draw_h_line(pos.y, decompressed, palette);
                // ili9341_write(&pos, decompressed);
                // next line...
                pos.y++;
                totalPixels += info_header.width;
                pixel = 0;
                nblines++;
            }
        }

        
        // else if (pixel > info_header.width)
        // {
        //     printf("!");
        //     //  pixel = 0;
        // }

        if (totalPixels > (info_header.width * info_header.height))
        {
           end = false; // error
        }
    }
    while((offset < fileSize) && !end);

    f_close(&fil);

    printf("\r\nNb lines :%d\r\nTotal pixels: %d", (uint32_t)nblines, (uint32_t)totalPixels);
}



static FATFS fs;


void configure_gpio()
{
      // Init DC GPIO
  rcu_periph_clock_enable(DC_GPIO_PORT);

  /* D/C (PA1) GPIO pin configuration  */
  gpio_init(DC_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, DC_PIN);

}

int main(void)
{
    configure_gpio();

    longan_led_init();
    init_uart0();
    longan_bp1_init();

    spi0_initialize(0);
    spi0_set_fclk_fast();
    ili9341_read_id();
    ili9341_initialize();
    ili9341_set_rotation(1);

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


  // DAC MCLK out from CK_OUT0 (PA8)
    // gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    // rcu_ckout0_config(RCU_CKOUT0SRC_CKPLL2_DIV2);

    decompress();
    // ili9341_fill();

    audio_init();

    audio_play(filename);

  // prepare_audio_buf();

    while(1)
    {
    //    run_audio_buf();
    }
    /*

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
    */
}
