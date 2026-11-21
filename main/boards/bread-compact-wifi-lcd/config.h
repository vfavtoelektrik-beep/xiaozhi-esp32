#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6

#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16
#endif

// Кнопка сна/разговора в пузе (GPIO 10)
#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_10
#define TOUCH_BUTTON_GPIO       GPIO_NUM_NC
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

// Ключ питания периферии (оптопара + FDD306P на ленту и усилитель)
#define PERIPH_POWER_PIN        GPIO_NUM_18

// Подсветка шлема (12 светодиодов WS2812B на GPIO 48)
#define NEOPIXEL_STRIP_PIN      GPIO_NUM_48
#define NEOPIXEL_STRIP_COUNT    12

// Контроль батареи 26650 (делитель 100к + 100к на GPIO 1)
#define BATTERY_ADC_PIN         GPIO_NUM_1

// Шина I2C для датчика BMP280 (GPIO 8 = SDA, GPIO 9 = SCL)
#define SENSOR_I2C_SDA_PIN      GPIO_NUM_8
#define SENSOR_I2C_SCL_PIN      GPIO_NUM_9

// Дисплей ST7789 1.3" 240x240 (7PIN без CS, разворот 180 градусов, BLK на GPIO 17)
#define DISPLAY_BACKLIGHT_PIN   GPIO_NUM_17
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#define DISPLAY_SPI_SCLK_PIN    GPIO_NUM_12
#define DISPLAY_SPI_MOSI_PIN    GPIO_NUM_11
#define DISPLAY_SPI_CS_PIN      GPIO_NUM_NC
#define DISPLAY_SPI_DC_PIN      GPIO_NUM_13
#define DISPLAY_SPI_RESET_PIN   GPIO_NUM_14

#define DISPLAY_WIDTH           240
#define DISPLAY_HEIGHT          240
#define DISPLAY_MIRROR_X        true
#define DISPLAY_MIRROR_Y        true
#define DISPLAY_SWAP_XY         false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER       LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X        0
#define DISPLAY_OFFSET_Y        0
#define DISPLAY_SPI_MODE        3

#endif // _BOARD_CONFIG_H_
