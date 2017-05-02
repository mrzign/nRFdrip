
#include "nrf_gpio.h"

#define LEDS_NUMBER    3

#ifdef BOARD_PCA10031
// LEDS
#define LED_START      21
#define LED_RGB_RED    21
#define LED_RGB_GREEN  22
#define LED_RGB_BLUE   23
#define LED_STOP       23

// UART connection with J-Link
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

// SPI
#define SPI0_CONFIG_SS_PIN         	15
#define SPI0_CONFIG_SCK_PIN         16
#define SPI0_CONFIG_MOSI_PIN        18
#define SPI0_CONFIG_MISO_PIN        17
#endif

#ifdef BOARD_NRFDRIP_P1A
// LEDS
#define LED_START      0
#define LED_RGB_RED    1
#define LED_RGB_GREEN  2
#define LED_RGB_BLUE   0
#define LED_STOP       2

// SPI
#define SPI0_CONFIG_SS_PIN         	16
#define SPI0_CONFIG_SCK_PIN         11
#define SPI0_CONFIG_MOSI_PIN    	15
#define SPI0_CONFIG_MISO_PIN        12

// ADC
#define AD_VBAT_EN_PIN				9

#endif


#define LED_RGB_RED_MASK    (1<<LED_RGB_RED)
#define LED_RGB_GREEN_MASK  (1<<LED_RGB_GREEN)
#define LED_RGB_BLUE_MASK   (1<<LED_RGB_BLUE)

#define LEDS_LIST { LED_RGB_RED, LED_RGB_GREEN, LED_RGB_BLUE}
// defining RGB led as 3 single LEDs
#define BSP_LED_0 LED_RGB_RED
#define BSP_LED_1 LED_RGB_GREEN
#define BSP_LED_2 LED_RGB_BLUE

#define BSP_LED_0_MASK    (1<<BSP_LED_0)
#define BSP_LED_1_MASK    (1<<BSP_LED_1)
#define BSP_LED_2_MASK    (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
//defines which LEDs are lit when signal is low
#define LEDS_INV_MASK  LEDS_MASK

// there are no user buttons
#define BUTTONS_NUMBER 0
#define BUTTONS_LIST {}
#define BUTTONS_MASK   0x00000000

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM


#define LEDS_OFF(leds_mask) do {  NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                            NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LEDS_ON(leds_mask) do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                           NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

#define LED_IS_ON(leds_mask) ((leds_mask) & (NRF_GPIO->OUT ^ LEDS_INV_MASK) )

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)
#ifdef BOARD_PCA10031
#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)
#endif

#ifdef BOARD_NRFDRIP_P1A

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
									  	  NRF_GPIO->PIN_CNF[pin] = ((GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) \
										  	  	  	  	  | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) \
                                      	  	  	  	  	  | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos) \
														  | (GPIO_PIN_CNF_DRIVE_H0S1 << GPIO_PIN_CNF_DRIVE_Pos) \
														  | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) ); } while(0)
#endif
