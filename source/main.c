/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file main.c
 *
 * @
 * @
 * @
 * @brief nRF51x22 implementation of xDrip/xBridge2
 *
 * @ref
 */
#include "hw_config.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_adc.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_config.h"
#include "nrf_delay.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf51_deprecated.h"
#include "cc2500.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"

#include "nrf_adc.h"
typedef __uint32_t uint32_t ;

#define TX_RX_MSG_LENGTH        		24

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                       	/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_CHAR_LEN              	20                                          /**< Max Size of the characteristic value being notified (in bytes). */

//#define TX_POWER_LEVEL			  	(-8)
#define APP_TIMER_PRESCALER           	0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE       	4                                           /**< Size of timer operation queues. */

#define CONNECTABLE_ADV_INTERVAL      	MSEC_TO_UNITS(100, UNIT_0_625_MS)           /**< The advertising interval for connectable advertisement (100 ms). This value can vary between 20ms to 10.24s. */
#define CONNECTABLE_ADV_TIMEOUT       	30                                          /**< Time for which the device must be advertising in connectable mode (in seconds). */

#define BLE_LED_ON_INTERVAL_IN_TKS	  	APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)
#define BLE_LED_OFF_INTERVAL_IN_TKS   	APP_TIMER_TICKS(1800, APP_TIMER_PRESCALER)

#define DEX_LED_ON_INTERVAL_IN_TKS	  	APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)
#define DEX_LED_OFF_INTERVAL_IN_TKS   	APP_TIMER_TICKS(2950, APP_TIMER_PRESCALER)

#define MIN_CONN_INTERVAL             	MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL             	MSEC_TO_UNITS(40, UNIT_1_25_MS)
#define SLAVE_LATENCY                 	0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT              	MSEC_TO_UNITS(6000, UNIT_10_MS)             /**< Connection supervisory timeout (6 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define COMPANY_IDENTIFIER            	0x0059                                      /**< Company identifier for Nordic Semiconductor ASA as per www.bluetooth.org. */

#define LOCAL_SERVICE_UUID            	0xFFE0                                      /**< Proprietary UUID for local service. */
#define LOCAL_CHAR_UUID               	0xFFE1                                      /**< Proprietary UUID for local characteristic. */

#define DEAD_BEEF                     	0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define ADC_FULLY_CHARGED				950 // >3.8V
#define ADC_FULLY_DRAINED				750 // <3.2V
#define ADC_CRITICAL					710 // <3.0V

#define SLEEP_TIME						299950 //~5min in ms (trimmed value)


static ble_gatts_char_handles_t m_char_handles;                                     /**< Handles of local characteristic (as provided by the BLE stack).*/
static uint16_t                 m_conn_handle = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
static uint16_t                 m_service_handle;                                   /**< Handle of local service (as provided by the BLE stack).*/
static bool                     m_is_notifying_enabled = false;                     /**< Variable to indicate whether the notification is enabled by the peer.*/

static ble_uuid_t m_adv_uuids[] = {{LOCAL_SERVICE_UUID, BLE_UUID_TYPE_BLE}}; 		/**< Universally unique service identifiers. */

volatile int32_t adc_batt_level;
volatile uint32_t adc_filtered_batt_level;
volatile bool adc_conv_ongoing = false;

static uint8_t m_tx_data_spi[TX_RX_MSG_LENGTH]; ///< SPI master TX buffer.
static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; ///< SPI master RX buffer.

static const nrf_drv_spi_t 		m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);



volatile bool					is_ble_advertising = false;
volatile bool					is_ble_connected = false;
volatile bool					wake_up_flag = false;
volatile bool					timeout = false;
volatile bool					ble_received_tx_id = false;
volatile bool					batt_critical = false;


volatile uint32_t dex_tx_id;

volatile uint8_t needsTimingCalibration = 1;
volatile uint8_t sequential_missed_packets = 0;
volatile uint8_t last_channel = 0;
volatile uint8_t got_packet = 0;

static uint8_t nChannels[4] = {0,100,199,209};

static int8_t fOffset[4] = {0x03, 0x04, 0x03, 0x02};
static int8_t defaultfOffset[4] = {0x03, 0x04, 0x03, 0x02};

volatile uint32_t waitTimes[4] = { 3500, 500, 500, 1500 };
volatile uint32_t defaultWaitTimes[4] = { 3500, 500, 500, 1500 };
static uint32_t delayedWaitTimes[4] = { 75000, 75000, 75000, 75000 };

volatile uint8_t nbr_received_pkts = 0;
volatile uint32_t ms_timestamp = 0;

volatile uint32_t ms_sleeptime = SLEEP_TIME;

volatile int32_t debug_timestamp[3][5] = {{ 0,  0,  0,  0,  0 }, { 0,  0,  0,  0,  0 }, { 0,  0,  0,  0,  0 }};

static const uint8_t SrcNameTable[32] = { 	'0', '1', '2', '3', '4', '5', '6', '7',
											'8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
											'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
											'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
};

APP_TIMER_DEF(m_ble_led_timer_id);			/**< LED timer. */
APP_TIMER_DEF(m_sleep_timer_id);			/**< Sleep timer. */
APP_TIMER_DEF(m_timeout_timer_id);			/**< Timeout timer. */
APP_TIMER_DEF(m_timestamp_timer_id);		/**< Timestamp timer. */

typedef struct _dex_pkt {
    uint8_t   pkt_len;
    uint32_t  dest_addr;
    uint32_t  src_addr;
    uint8_t   port;
    uint8_t   device_info; //count
    uint8_t   tx_id;
    uint16_t  raw_data;
    uint16_t  filtered_data;
    uint8_t   battery;
    uint8_t   fcs;
    uint8_t   crc;
    int8_t    rssi;
    uint8_t   lqi;
} dex_pkt;

static uint8_t                  m_char_value[sizeof(dex_pkt)];                     /**< Value of the BLE characteristic */

uint8_t bit_reverse_byte(uint8_t in) {
    uint8_t bRet = 0;
    if(in & 0x01) bRet |= 0x80;
    if(in & 0x02) bRet |= 0x40;
    if(in & 0x04) bRet |= 0x20;
    if(in & 0x08) bRet |= 0x10;
    if(in & 0x10) bRet |= 0x08;
    if(in & 0x20) bRet |= 0x04;
    if(in & 0x40) bRet |= 0x02;
    if(in & 0x80) bRet |= 0x01;
    return bRet;
}

uint8_t min8(uint8_t a, uint8_t b) {
    if(a < b) return a;
    return b;
}

void bit_reverse_bytes(uint8_t* buf, uint8_t nLen) {
	for(uint8_t i = 0; i < nLen; i++) {
        buf[i] = bit_reverse_byte(buf[i]);
    }
}

uint32_t dex_num_decoder(uint16_t usShortFloat) {
    uint16_t usReversed = usShortFloat;
    uint8_t usExponent = 0;
    uint32_t usMantissa = 0;
    bit_reverse_bytes((uint8_t*)&usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

uint32_t getSrcValue(uint8_t srcVal) {
	uint8_t i = 0;

	for (i = 0; i < 32; i++)
	{
		if (SrcNameTable[i] == srcVal)
			break;
	}
	return i & 0xFF;
}

uint32_t ascii_to_dex_src(const uint8_t addr[6])
{
	uint32_t src = 0;
	src |= (getSrcValue(addr[0]) << 20);
	src |= (getSrcValue(addr[1]) << 15);
	src |= (getSrcValue(addr[2]) << 10);
	src |= (getSrcValue(addr[3]) << 5);
	src |=  getSrcValue(addr[4]);
	return src;
}
/*
void dex_src_to_ascii(void) {
	dynamic_transmitter_id[0] = SrcNameTable[(dex_tx_id >> 20) & 0x1F];
	dynamic_transmitter_id[1] = SrcNameTable[(dex_tx_id >> 15) & 0x1F];
	dynamic_transmitter_id[2] = SrcNameTable[(dex_tx_id >> 10) & 0x1F];
	dynamic_transmitter_id[3] = SrcNameTable[(dex_tx_id >> 5) & 0x1F];
	dynamic_transmitter_id[4] = SrcNameTable[(dex_tx_id >> 0) & 0x1F];
	dynamic_transmitter_id[5] = 0;
}*/

void reset_offsets() {
    for(uint8_t i=0; i<4; i++) {
        fOffset[i] = defaultfOffset[i];
    }
}

static void timeout_timer_handler(void * p_context)
{
	timeout = true;
}


static void spi_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config =
    {
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .orc          = 0xFF,
        .frequency    = NRF_SPI_FREQ_1M,
        .mode         = NRF_SPI_MODE_0,
        .bit_order    = NRF_SPI_BIT_ORDER_MSB_FIRST,
    };

	config.ss_pin	= SPI0_CONFIG_SS_PIN;
	config.sck_pin  = SPI0_CONFIG_SCK_PIN;
	config.mosi_pin = SPI0_CONFIG_MOSI_PIN;
	config.miso_pin = SPI0_CONFIG_MISO_PIN;
	err_code = nrf_drv_spi_init(&m_spi_master_0, &config, NULL);
	APP_ERROR_CHECK(err_code);
}




/**@brief Function for sending and receiving data.
 *
 * @param[in]   p_instance   Pointer to SPI master driver instance.
 * @param[in]   p_tx_data    A pointer to a buffer TX.
 * @param[out]  p_rx_data    A pointer to a buffer RX.
 * @param[in]   len          A length of the data buffers.
 */
static void spi_send_recv(nrf_drv_spi_t const * p_instance,
                          uint8_t * p_tx_data,
                          uint8_t * p_rx_data,
                          uint16_t  len)
{

    uint32_t err_code = nrf_drv_spi_transfer(p_instance, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);

}


uint8_t cc2500_read_reg(uint8_t addr)
{
/*
 * Address
 * 7		6		5		4		3		2		1		0
 * R=1/W=0	Burst	A5		A4		A3		A2		A1		A0
 */

	//Read from register address
	addr = addr | 0x80;

	uint16_t len = 2;
    m_tx_data_spi[0] = addr;

	spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);
	return m_rx_data_spi[1];

}

uint8_t* cc2500_read_burst_reg(uint8_t addr, uint8_t count)
{
/*
 * Address
 * 7		6		5		4		3		2		1		0
 * R=1/W=0	Burst	A5		A4		A3		A2		A1		A0
 */
	//Read from registers, burst mode
	addr = addr | 0xC0;

	uint16_t len = count + 1;
    m_tx_data_spi[0] = addr;

	spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);

	return (uint8_t*)&m_rx_data_spi[1];
}

void cc2500_get_dex_pkt(uint8_t addr, dex_pkt* p_pkt, uint8_t pkt_len)
{
/*
 * Address
 * 7		6		5		4		3		2		1		0
 * R=1/W=0	Burst	A5		A4		A3		A2		A1		A0
 */
	//Read from registers, burst mode
	addr = addr | 0xC0;

	uint16_t len = pkt_len + 1;
    m_tx_data_spi[0] = addr;

	spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);

	p_pkt->pkt_len = m_rx_data_spi[1];
	p_pkt->dest_addr = (uint32_t)(m_rx_data_spi[5] << 24 | m_rx_data_spi[4] << 16 | m_rx_data_spi[3] << 8 | m_rx_data_spi[2]);
	p_pkt->src_addr = (uint32_t)(m_rx_data_spi[9] << 24 | m_rx_data_spi[8] << 16 | m_rx_data_spi[7] << 8 | m_rx_data_spi[6]);
	p_pkt->port = m_rx_data_spi[10];
	p_pkt->device_info = m_rx_data_spi[11]; //count
	p_pkt->tx_id = m_rx_data_spi[12];
	p_pkt->raw_data = (uint16_t)(m_rx_data_spi[14] << 8 | m_rx_data_spi[13]);
	p_pkt->filtered_data = (uint16_t)(m_rx_data_spi[16] << 8 | m_rx_data_spi[15]);
	p_pkt->battery = m_rx_data_spi[17];
	p_pkt->fcs = m_rx_data_spi[18];
	p_pkt->crc = m_rx_data_spi[19];
	p_pkt->rssi = m_rx_data_spi[20];
	p_pkt->lqi = m_rx_data_spi[21];

}

uint8_t cc2500_read_status_reg(uint8_t addr)
{
	uint16_t len = 2;

	addr = addr | 0xC0;

    m_tx_data_spi[0] = addr;
    m_tx_data_spi[1] = 0x00;

    spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);

	return m_rx_data_spi[1];
}

uint8_t cc2500_send_strobe(uint8_t strobe)
{
	uint16_t len = 1;
    m_tx_data_spi[0] = strobe;

	spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);
	return m_rx_data_spi[0];
}

uint8_t cc2500_write_reg(uint8_t addr, uint8_t value)
{
	uint16_t len = 2;
    m_tx_data_spi[0] = addr;
    m_tx_data_spi[1] = value;

	spi_send_recv(&m_spi_master_0, m_tx_data_spi, m_rx_data_spi, len);
	return m_rx_data_spi[0];
}

void cc2500_reset()
{
	cc2500_send_strobe(CC2500_CMD_SRES);
}

void cc2500_swap_channel(uint8_t channel, uint8_t newFSCTRL0)
{
	do
	{
		cc2500_send_strobe(CC2500_CMD_SIDLE);
	} while ((cc2500_read_status_reg(CC2500_REG_MARCSTATE) & 0x1F) != 0x01);

	cc2500_write_reg(CC2500_REG_CHANNR, channel);
	cc2500_write_reg(CC2500_REG_FSCTRL0, newFSCTRL0);

	cc2500_send_strobe(CC2500_CMD_SRX);
	while ((cc2500_read_status_reg(CC2500_REG_MARCSTATE) & 0x1F) != 0x0D) {};
}

/** @brief Function for initial configuration of cc2500 registers.
 */
void cc2500_init(void)
{
	cc2500_reset();

												//Register settings - (Default) , [xBridge]
	cc2500_write_reg(CC2500_REG_PATABLE, 0x00); //(0x00)
	//cc2500_write_reg(CC2500_REG_IOCFG2, 0x29) //(0x29) - GDO2, CHIP_RDYn
	cc2500_write_reg(CC2500_REG_IOCFG1, 0x01);	//(0x2e) - GDO1,  Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end of packet is reached. De-asserts when the RX FIFO is empty
	//cc2500_write_reg(CC2500_REG_IOCFG0, 0x3F) //(0x3F) - GDO0, CLK_XOSC/192

	cc2500_write_reg(CC2500_REG_PKTLEN, 0x12);	//(0xFF)
	cc2500_write_reg(CC2500_REG_PKTCTRL1, 0x04);//(0x04)
	cc2500_write_reg(CC2500_REG_PKTCTRL0, 0x05);//(0x45) - No Whitening

	cc2500_write_reg(CC2500_REG_ADDR, 0x00);	//(0x00)
	cc2500_write_reg(CC2500_REG_CHANNR, 0x00);	//(0x00)

	cc2500_write_reg(CC2500_REG_FSCTRL1, 0x0a);	//(0x0F)[0x0a]
	cc2500_write_reg(CC2500_REG_FSCTRL0, 0x00);	//(0x00)

	cc2500_write_reg(CC2500_REG_FREQ2, 0x5d);	//(0x5E) - 2424.999512MHz [2424.999756]
	cc2500_write_reg(CC2500_REG_FREQ1, 0x44);	//(0xC4)
	cc2500_write_reg(CC2500_REG_FREQ0, 0xeb);	//(0xEC)

	cc2500_write_reg(CC2500_REG_FREND1, 0xb6);	//(0x56) - LNA Current
	cc2500_write_reg(CC2500_REG_FREND0, 0x10);	//(0x10)

	cc2500_write_reg(CC2500_REG_MDMCFG4, 0x5a); //(0x8C) - 325 kHz RX Filter BW [375kHz]
	cc2500_write_reg(CC2500_REG_MDMCFG3, 0xf8); //(0x22) - 49.9878 kBaud
	cc2500_write_reg(CC2500_REG_MDMCFG2, 0x73);	//(0x02) - MSK
	cc2500_write_reg(CC2500_REG_MDMCFG1, 0x03);	//(0x22) - 249.938965 kHz Channel Spacing [249.755859]
	cc2500_write_reg(CC2500_REG_MDMCFG0, 0x3b); //(0xF8)

	cc2500_write_reg(CC2500_REG_DEVIATN, 0x40); //(0x47) - Phase Transition Time = 0

	cc2500_write_reg(CC2500_REG_MCSM2, 0x07);	//(0x07)
	cc2500_write_reg(CC2500_REG_MCSM1, 0x30);	//(0x30)
	cc2500_write_reg(CC2500_REG_MCSM0, 0x18);	//(0x04) - AutoCal when going from IDLE to RX or TX, PO_TIMEOUT = 2

	cc2500_write_reg(CC2500_REG_FOCCFG, 0x0A);	//(0x36) - FOC_PRE_K = 1 // allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE

	cc2500_write_reg(CC2500_REG_FSCAL3, 0xa9);	//(0xA9)
	cc2500_write_reg(CC2500_REG_FSCAL2, 0x0a);	//(0x0A)
	cc2500_write_reg(CC2500_REG_FSCAL1, 0x00);	//(0x20) - SmartRF output
	cc2500_write_reg(CC2500_REG_FSCAL0, 0x11);	//(0x0D) - SmartRF output

	cc2500_write_reg(CC2500_REG_AGCCTRL2, 0x03);//(0x03) - [0x44, MAX_DVGA_GAIN = The highest gain setting can not be used, MAGN_TARGET = 4 = 36dB]
	cc2500_write_reg(CC2500_REG_AGCCTRL1, 0x00);//(0x40) - LNA2 gain is decreased to minimum before decreasing LNA gain
	cc2500_write_reg(CC2500_REG_AGCCTRL0, 0x91);//(0x91) - [0xB2, WAIT_TIME, FILTER_LENGTH]
	cc2500_write_reg(CC2500_REG_BSCFG, 0x6C);	//(0x6C)

}

/** @brief Function for configuration of cc2500 registers after sleep.
 */
void cc2500_reinit(void)
{
	//Registers that Lose Programming in SLEEP State
	cc2500_write_reg(CC2500_REG_TEST2, 0x81);	//(0x88) - Increased sensitivity for data rates <100kBaud [0x88]
	cc2500_write_reg(CC2500_REG_TEST1, 0x35);	//(0x31) - Increased sensitivity for data rates <100kBaud [0x31]
	cc2500_write_reg(CC2500_REG_TEST0, 0x0b);	//(0x0B) - SmartRF output
}

void cc2500_sleep(void)
{
	//Put radio to sleep
	do
	{
		cc2500_send_strobe(CC2500_CMD_SIDLE);
		nrf_delay_ms(10);
	} while ((cc2500_read_status_reg(CC2500_REG_MARCSTATE) & 0x1F) != 0x01);
	cc2500_send_strobe(CC2500_CMD_SPWD);
}

uint8_t WaitForPacket(uint32_t milliseconds, dex_pkt* p_pkt, uint8_t channel)
{
	uint32_t err_code;
    uint8_t pkt_len = 0;
    uint8_t result = 0;

	cc2500_swap_channel(nChannels[channel], fOffset[channel]);

	timeout = false;
    err_code = app_timer_start(m_timeout_timer_id, APP_TIMER_TICKS(milliseconds, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

    while (!timeout && !result)
    {
    	//Do not spam CC2500 with SPI polling.
    	//GDO1 asserts when end of packet is reached.
    	//GDO1 De-asserts when the RX FIFO is empty.
    	if(nrf_gpio_pin_read(SPI0_CONFIG_MISO_PIN) == 1)
    	{
    		pkt_len = cc2500_read_status_reg(CC2500_REG_RXBYTES);
    	}

		if(pkt_len == 0x15)
		{
			//Store for debug
			debug_timestamp[0][nbr_received_pkts] = ms_timestamp;
			debug_timestamp[1][nbr_received_pkts] = channel;

			cc2500_get_dex_pkt(CC2500_REG_RXFIFO, p_pkt, pkt_len);

			if(cc2500_read_status_reg(CC2500_REG_LQI) & 0x80)	// Check if CRC passed
			{
				//Store estimated freq. offset.
				fOffset[channel] += cc2500_read_status_reg(CC2500_REG_FREQEST);

                if(p_pkt->src_addr == dex_tx_id)// || dex_tx_id == 0)
                {
                	//It's for us
    				//Reset timestamp for accurate sleep time calculation
    		        ms_timestamp = 0;
    		        last_channel = channel;
					result = 1;
                }
			}
			else
			{
				//CRC-error
				//LEDS_ON(LED_RGB_BLUE_MASK);
			}

        }

		if(pkt_len > 0 && !result)
		{
			pkt_len = 0;

			//Flush RX FIFO since it contains corrupt data or a pkg not ment for us
			do
			{
				cc2500_send_strobe(CC2500_CMD_SIDLE);
				nrf_delay_ms(10);
			} while ((cc2500_read_status_reg(CC2500_REG_MARCSTATE) & 0x1F) != 0x01);

			cc2500_send_strobe(CC2500_CMD_SFRX);

			//Re-enable RX
			cc2500_send_strobe(CC2500_CMD_SRX);
			while ((cc2500_read_status_reg(CC2500_REG_MARCSTATE) & 0x1F) != 0x0D) {};
		}

    }

    err_code = app_timer_stop(m_timeout_timer_id);
	APP_ERROR_CHECK(err_code);

    return result;
}

void resetWaitTimes(void)
{
	for(uint8_t i=0; i<4; i++)
	{
		waitTimes[i] = defaultWaitTimes[i];
	}
}

uint32_t delayFor(uint8_t wait_chan)
{
    if(needsTimingCalibration)
    {
        return delayedWaitTimes[wait_chan];
    }
    return waitTimes[wait_chan];
}


uint8_t get_packet(dex_pkt* p_pkt)
{
	for(uint8_t nChannel = 0; nChannel < 4; nChannel++)
    {
        switch(WaitForPacket(delayFor(nChannel), p_pkt, nChannel))
        {
			case 1:
				if(sequential_missed_packets > 2)
				{
					sequential_missed_packets = 0;
					resetWaitTimes();
				}
				needsTimingCalibration = 0;
				return 1;
			case 0:
				//We timed out. Try next channel
				continue;
        }
    }
    sequential_missed_packets ++;

    if(sequential_missed_packets > 2)
    {
    	waitTimes[0] += 500;
    	waitTimes[3] += 1000;
    }
    if(sequential_missed_packets > 20)
    {
    	resetWaitTimes();
        sequential_missed_packets = 0;
        needsTimingCalibration = 1;
    }

    reset_offsets();
    return 0;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	LEDS_OFF(LEDS_MASK);
	LEDS_ON(LED_RGB_RED_MASK);
	nrf_delay_ms(10000);
	NVIC_SystemReset();
    //app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**
 * @brief ADC interrupt handler.
 */
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();
    //Get value
    adc_batt_level = nrf_adc_result_get();
    adc_conv_ongoing = false;

}

/**
 * @brief Return battery capacity.
 */
int8_t battery_capacity(void)
{
	int8_t batt_level = 0;
	//1024 = 4.3333V
	if(adc_filtered_batt_level > ADC_FULLY_CHARGED)
		batt_level = 100;
	else if(adc_filtered_batt_level < 100)
		batt_level = -1;
	else if(adc_filtered_batt_level < ADC_FULLY_DRAINED)
		batt_level = 1;
	else
		batt_level = 100 * (adc_filtered_batt_level - ADC_FULLY_DRAINED) / (ADC_FULLY_CHARGED - ADC_FULLY_DRAINED);
	return batt_level;

}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	LEDS_OFF(LEDS_MASK);
	LEDS_ON(LED_RGB_RED_MASK);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
        is_ble_advertising = false;
    }
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Characteristic notification.
 *
 * @details Sends one characteristic value notification to peer if connected to it and the
 *          notification is enabled.
 */
static uint8_t ble_send_dex_pkt(dex_pkt* p_pkt)
{
    uint32_t err_code;
    uint16_t len;

    // Send value if connected and notifying.

    if ((m_conn_handle != BLE_CONN_HANDLE_INVALID) && m_is_notifying_enabled)
    {

    	ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle   = m_char_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;

    	//Prepare BLE package to send
        /*xBridge2 package format

        	uint8_t		size;		//size of the packet.
        	uint8_t		cmd_code;	//code for this data packet.  Always 00 for a Dexcom data packet.
        	uint32_t	raw;		//"raw" BGL value.
        	uint32_t	filtered;	//"filtered" BGL value
        	uint8_t		dex_battery;//battery value
        	int8_t		my_battery;	//xBridge battery value
        	uint32_t	dex_src_id;	//raw TXID of the Dexcom Transmitter
        	uint8_t		function; 	//Byte representing the xBridge code funcitonality.  01 = this level.
        */
        m_char_value[0]		= 17;
        m_char_value[1]		= 0x00;
    	uint32_t raw		= dex_num_decoder(p_pkt->raw_data);
    	m_char_value[2]		= (raw & 0x000000FF) >> 0;
    	m_char_value[3]		= (raw & 0x0000FF00) >> 8;
    	m_char_value[4]		= (raw & 0x00FF0000) >> 16;
    	m_char_value[5]		= (raw & 0xFF000000) >> 24;
    	uint32_t filtered	= dex_num_decoder(p_pkt->filtered_data)*2;
    	m_char_value[6]		= (filtered & 0x000000FF) >> 0;
    	m_char_value[7]		= (filtered & 0x0000FF00) >> 8;
    	m_char_value[8]		= (filtered & 0x00FF0000) >> 16;
    	m_char_value[9]		= (filtered & 0xFF000000) >> 24;
    	m_char_value[10] 	= p_pkt->battery;
    	m_char_value[11]	= battery_capacity();
    	uint32_t dex_src_id = dex_tx_id; //p_pkt->src_addr not used since we need the tx_id from apk
    	m_char_value[12]	= (dex_src_id & 0x000000FF) >> 0;
    	m_char_value[13]	= (dex_src_id & 0x0000FF00) >> 8;
    	m_char_value[14]	= (dex_src_id & 0x00FF0000) >> 16;
    	m_char_value[15]	= (dex_src_id & 0xFF000000) >> 24;
    	m_char_value[16]	= 0x01;

        len = m_char_value[0];

        hvx_params.p_len    = &len;
        hvx_params.p_data   = m_char_value;

        err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            return 1;
        }
    }
    return 0;

}

static uint8_t ble_send_beacon(void)
{
    uint32_t err_code;
    uint16_t len;

    // Send value if connected and notifying.

    if ((m_conn_handle != BLE_CONN_HANDLE_INVALID) && m_is_notifying_enabled)
    {

    	ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle   = m_char_handles.value_handle;
        hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;

    	//Prepare BLE package to send
        /*xBridge2 "TX ID request" package format

        	uint8_t		size;		//size of the packet.
        	uint8_t		cmd_code;	//code for this data packet.  Always F1 for "beacon" data packet.
        	uint32_t	dex_tx_id;	//raw TXID of the Dexcom Transmitter
        	uint8_t		function; 	//byte representing the xBridge code functionality.  01 = this level.
        */

    	m_char_value[0] = 7;
    	m_char_value[1] = 0xF1;
    	m_char_value[2]	= (dex_tx_id & 0x000000FF) >> 0;
    	m_char_value[3]	= (dex_tx_id & 0x0000FF00) >> 8;
    	m_char_value[4]	= (dex_tx_id & 0x00FF0000) >> 16;
    	m_char_value[5]	= (dex_tx_id & 0xFF000000) >> 24;
    	m_char_value[6] = 0x01;

        len = m_char_value[0];

        hvx_params.p_len    = &len;
        hvx_params.p_data   = m_char_value;

        err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            return 1;
        }
    }
    return 0;

}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //Turn on adv. led
        	is_ble_advertising = true;
            err_code = app_timer_start(m_ble_led_timer_id, BLE_LED_OFF_INTERVAL_IN_TKS, NULL);
        	APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
        	//Turn off adv. led
        	is_ble_advertising = false;
            LEDS_OFF(LED_RGB_BLUE_MASK);
            break;

        default:
            break;
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions.
 *
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t			ble_addr;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_address_get(&ble_addr);
    APP_ERROR_CHECK(err_code);
    uint8_t dev_name[12] = "nRFdrip_";
    //Add last two bytes of address as unique suffix to device name
    dev_name[8] = ((ble_addr.addr[1] & 0xF0) >> 4);
    dev_name[8] = (dev_name[8] > 0x09) ? dev_name[8] + 0x37 : dev_name[8] + 0x30;
    dev_name[9] = ((ble_addr.addr[1] & 0x0F));
    dev_name[9] = (dev_name[9] > 0x09) ? dev_name[9] + 0x37 : dev_name[9] + 0x30;
    dev_name[10] = ((ble_addr.addr[0] & 0xF0) >> 4);
    dev_name[10] = (dev_name[10] > 0x09) ? dev_name[10] + 0x37 : dev_name[10] + 0x30;
    dev_name[11] = ((ble_addr.addr[0] & 0x0F));
    dev_name[11] = (dev_name[11] > 0x09) ? dev_name[11] + 0x37 : dev_name[11] + 0x30;

    err_code = sd_ble_gap_device_name_set(&sec_mode, dev_name, sizeof(dev_name));

    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    // Set GAP Peripheral Preferred Connection Parameters (converting connection interval from
    // milliseconds to required unit of 1.25ms).
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
//    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertisement packet.
 *
 * @details This function initializes the data that is to be placed in an advertisement packet in
 *          both connectable and non-connectable modes.
 *
 */
static void advertising_data_init(void)
{
    uint32_t		err_code;
    ble_advdata_t	advdata;
    ble_advdata_t 	scanrsp;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

	advdata.flags                 	= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.include_appearance 		= false;
    advdata.name_type 				= BLE_ADVDATA_FULL_NAME;


    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options 	= {0};
    options.ble_adv_fast_enabled  	= BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval 	= CONNECTABLE_ADV_INTERVAL;
    options.ble_adv_fast_timeout  	= CONNECTABLE_ADV_TIMEOUT;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for adding the Characteristic.
 *
 * @details This function adds the characteristic to the local db.
 *
 * @param[in] uuid_type Type of service UUID assigned by the S110 SoftDevice.
 *
 */
static void char_add(const uint8_t uuid_type)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    memset(&char_md, 0, sizeof(char_md));


    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    char_uuid.type = uuid_type;
    char_uuid.uuid = LOCAL_CHAR_UUID;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = APP_CFG_CHAR_LEN;

    attr_char_value.p_value   = NULL;

    err_code = sd_ble_gatts_characteristic_add(m_service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &m_char_handles);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for adding the Service.
 *
 * @details This function adds the service and the characteristic within it to the local db.
 *
 */
static void service_add(void)
{
    ble_uuid_t  service_uuid;
    uint32_t    err_code;

    BLE_UUID_BLE_ASSIGN(service_uuid, LOCAL_SERVICE_UUID);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &m_service_handle);
    APP_ERROR_CHECK(err_code);

    // Add characteristics
    char_add(service_uuid.type);

}


static void ble_led_timer_handler(void * p_context)
{
	uint32_t err_code;

    if(is_ble_advertising)
    {
		if (LED_IS_ON(LED_RGB_BLUE_MASK))
		{
			LEDS_OFF(LED_RGB_BLUE_MASK);
			err_code = app_timer_start(m_ble_led_timer_id, BLE_LED_OFF_INTERVAL_IN_TKS, NULL);
			APP_ERROR_CHECK(err_code);
		}
		else
		{
			LEDS_ON(LED_RGB_BLUE_MASK);
			err_code = app_timer_start(m_ble_led_timer_id, BLE_LED_ON_INTERVAL_IN_TKS, NULL);
			APP_ERROR_CHECK(err_code);
		}
    }

}

static void sleep_timer_handler(void * p_context)
{
	wake_up_flag = true;
}

static void timestamp_timer_handler(void * p_context)
{
	ms_timestamp++;
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the Timer initialization.
 *
* @details Initializes the timer module.
*/
static void timers_init(void)
{
    uint32_t err_code;

    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_timeout_timer_id,
    							APP_TIMER_MODE_SINGLE_SHOT,
								timeout_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ble_led_timer_id,
    							APP_TIMER_MODE_SINGLE_SHOT,
								ble_led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sleep_timer_id,
    							APP_TIMER_MODE_SINGLE_SHOT,
								sleep_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timestamp_timer_id,
    							APP_TIMER_MODE_REPEATED,
								timestamp_timer_handler);
    APP_ERROR_CHECK(err_code);

}

/**
 * @brief ADC initialization.
 */
void adc_config(void)
{
    nrf_adc_config_t nrf_adc_config;
    nrf_adc_config.resolution = NRF_ADC_CONFIG_RES_10BIT;
    nrf_adc_config.scaling = NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE;
    nrf_adc_config.reference = NRF_ADC_CONFIG_REF_VBG;

    // Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_4);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    NVIC_EnableIRQ(ADC_IRQn);

    //Configure enable gpio and put it inactive
    nrf_gpio_cfg_output(AD_VBAT_EN_PIN);
    nrf_gpio_pin_clear(AD_VBAT_EN_PIN);
}

void sample_battery(void)
{
	uint8_t i = 0;
	uint32_t err_code;

	timeout=false;
	err_code = app_timer_start(m_timeout_timer_id, APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

	//Sample battery level
	nrf_gpio_pin_set(AD_VBAT_EN_PIN);
	adc_filtered_batt_level = 0;
	for(i = 0; (i < 11 && !timeout); i++) {
		nrf_delay_ms(10);
		adc_conv_ongoing = true;
		nrf_adc_start();
		while(adc_conv_ongoing && !timeout);
		if(i>0) {
			adc_filtered_batt_level += adc_batt_level;
		}
	}
	err_code = app_timer_stop(m_timeout_timer_id);
	APP_ERROR_CHECK(err_code);

	nrf_adc_stop();
    nrf_gpio_pin_clear(AD_VBAT_EN_PIN);

	if(!timeout)
		adc_filtered_batt_level /= 10;

    //Determine if battery level is critical (and valid. If not connected(debug), do not consider as critical)
	if(adc_filtered_batt_level < ADC_CRITICAL && adc_filtered_batt_level > 100)
		batt_critical = true;
	else if(adc_filtered_batt_level > ADC_FULLY_DRAINED)
		batt_critical = false;
}

/**@brief Function for evaluating the value written in CCCD
 *
 * @details This shall be called when there is a write event received from the stack. This
 *          function will evaluate whether or not the peer has enabled notifications and
 *          start/stop notifications accordingly.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_write(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == m_char_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written. Start notifications
        m_is_notifying_enabled = ble_srv_is_notification_enabled(p_evt_write->data);

    }
    else if ((p_evt_write->handle == m_char_handles.value_handle) && (p_evt_write->len > 0))
    {
    	switch(p_evt_write->len)
    	{
			case 6:
				//New dex_tx_id received
				if(p_evt_write->data[0] == 0x06 && p_evt_write->data[1] == 0x01)
				{
					uint32_t rec_dex_tx_id;
					memcpy(&rec_dex_tx_id, &p_evt_write->data[2],sizeof(dex_tx_id));
					dex_tx_id = rec_dex_tx_id;
					ble_received_tx_id = true;
					LEDS_OFF(LED_RGB_RED_MASK);
				}
				break;

			case 2:
				//ACK received
				if(p_evt_write->data[0] == 0x02 && p_evt_write->data[1] == 0xF0)
				{
					//Really nothing to do here.. I will go to sleep anyway
				}
				break;

			default:
	            // No implementation needed.
	            break;
    	}
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            is_ble_advertising = false;
            is_ble_connected = true;
            LEDS_ON(LED_RGB_BLUE_MASK);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            is_ble_advertising = false;
            is_ble_connected = false;
            LEDS_OFF(LED_RGB_BLUE_MASK);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    //ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;

    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

}

void BLE_sleep(void)
{
	uint32_t err_code;

	//Stop connection parameters update timer, if running
	err_code = ble_conn_params_stop();
	APP_ERROR_CHECK(err_code);

	if(is_ble_connected)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		while(is_ble_connected){};
	}

	if(is_ble_advertising)
	{
		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);
		is_ble_advertising = false;
	}
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
	uint32_t err_code;

	//Bring down BLE-link
	BLE_sleep();

	//Turn off LEDS. But let them light first
	nrf_delay_ms(1000);
	LEDS_OFF(LEDS_MASK);

	err_code = app_timer_stop_all();
	APP_ERROR_CHECK(err_code);

	//Calculate sleep time
	//Time to sleep = 5 min - Time Elapsed between capture and now - (captured channel * channel time slot width)
	uint32_t sleeptime = ms_sleeptime - ms_timestamp;
	if(got_packet)
	{
		sleeptime -= defaultWaitTimes[0];
		//Compensate for channel
		for(uint8_t i=1; i <= last_channel; i++)
		{
			sleeptime -= 500;
		}
		//Aim at center of first channel
		sleeptime += 250;

	}

	wake_up_flag = false;

	//Check if reasonable sleep time
	if(sleeptime > 0 && sleeptime <= SLEEP_TIME)
	{
		err_code = app_timer_start( m_sleep_timer_id,
    							APP_TIMER_TICKS(sleeptime, APP_TIMER_PRESCALER),
								NULL);
		APP_ERROR_CHECK(err_code);

		while(!wake_up_flag)
		{
			__SEV();
			__WFE();
			__WFE();
		}

	}
	else
	{
		//Should not end up here...
		LEDS_ON(LED_RGB_RED_MASK);
		nrf_delay_ms(10000);
		NVIC_SystemReset();
	}

}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    //Turn on/off LEDS (self-test)
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
    LEDS_ON(LED_RGB_RED_MASK);
    nrf_delay_ms(300);
    LEDS_OFF(LEDS_MASK);
    nrf_delay_ms(100);
    LEDS_ON(LED_RGB_GREEN_MASK);
    nrf_delay_ms(300);
    LEDS_OFF(LEDS_MASK);
    nrf_delay_ms(100);
    LEDS_ON(LED_RGB_BLUE_MASK);
    nrf_delay_ms(300);
    LEDS_OFF(LEDS_MASK);
    nrf_delay_ms(100);

    timers_init();

    //Config ADC for battery measurement
    adc_config();

	//Init CC2500 RF front end
    spi_init();
	cc2500_init();

	//Read Chip ID (self-test)
	uint8_t part_num = cc2500_read_reg(CC2500_REG_PARTNUM);
	if(part_num != 0x80)
	{
		for(uint8_t i = 0; i < 5; i++) {
			LEDS_ON(LED_RGB_RED_MASK);
			nrf_delay_ms(50);
			LEDS_OFF(LEDS_MASK);
			nrf_delay_ms(200);
		}
	}

	//dex_tx_id = ascii_to_dex_src(transmitter_id);
	dex_tx_id = 0;

	if(dex_tx_id == 0)
	{
		//No valid dex_tx_id yet
		//Indicate fault condition
		LEDS_ON(LED_RGB_RED_MASK);
	}


	//Make first measurement
	sample_battery();

	while(batt_critical)
	{
		sample_battery();

		//Indicate low bat and goto sleep
		for(uint8_t i = 0; i < 3; i++) {
			LEDS_ON(LED_RGB_RED_MASK);
			nrf_delay_ms(50);
			LEDS_OFF(LEDS_MASK);
			nrf_delay_ms(200);
		}
		power_manage();
	}

	//Init BLE
	ble_stack_init();
    gap_params_init();
    service_add();
    advertising_data_init();
    conn_params_init();
    advertising_start();

    //Wait for BLE connection and GATT write flag (will not work without it)
    //Leave it on for user to configure xBridgeWixel + Dex Transmitter ID
    while(!m_is_notifying_enabled)
    {
    	nrf_delay_ms(1000);
    	if(!is_ble_connected && !is_ble_advertising)
    		advertising_start();
    }

    //Ask for dex_tx_id;
    while(!ble_received_tx_id)
    {
    	ble_send_beacon();
    	nrf_delay_ms(500);
    	if(!is_ble_connected && !is_ble_advertising)
    		advertising_start();
    }


    while (true)
    {
    	//Reset time stamp for timing calibration
    	ms_timestamp = 0;
    	err_code = app_timer_start(m_timestamp_timer_id, APP_TIMER_TICKS(1, APP_TIMER_PRESCALER),NULL);
    	APP_ERROR_CHECK(err_code);

    	//If battery level is or just were critical
    	if(batt_critical)
    	{
    		needsTimingCalibration = 1;
    		//Get new battery value
    		sample_battery();

			//Indicate low bat and goto sleep
			for(uint8_t i = 0; i < 3; i++) {
				LEDS_ON(LED_RGB_RED_MASK);
				nrf_delay_ms(50);
				LEDS_OFF(LEDS_MASK);
				nrf_delay_ms(200);
			}
			power_manage();
    	}
    	else
    	{
			//Re-init registers after SLEEP State
			cc2500_reinit();

			if(dex_tx_id == 0)
			{
				//No valid dex_tx_id
				//Indicate fault condition
				LEDS_ON(LED_RGB_RED_MASK);
			}

			//Prepare dex pkt
			dex_pkt pkt;
			memset(&pkt, 0, sizeof(dex_pkt));
			//Fetch dex pkt
			got_packet = get_packet(&pkt);

			//Put radio to sleep (power save)
			cc2500_sleep();

			//Update battery level with latest greatest
			sample_battery();

			if(got_packet)
			{
				//Yeay, we got a packet!
				LEDS_ON(LED_RGB_GREEN_MASK);

				//Keep track of last 4 pkts (debug)
				nbr_received_pkts++;
				if(nbr_received_pkts > 4)
				{
					nbr_received_pkts = 0;
				}

				//Wait for BLE link to establish (max 60s)
				timeout=false;
				err_code = app_timer_start(m_timeout_timer_id, APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER), NULL);
				APP_ERROR_CHECK(err_code);
				while(!is_ble_connected && !timeout)
				{
					if(!is_ble_connected && !is_ble_advertising)
					{
						advertising_start();
					}
					nrf_delay_ms(500);
				}
				err_code = app_timer_stop(m_timeout_timer_id);
				APP_ERROR_CHECK(err_code);

				if(!timeout)
				{
					err_code = app_timer_start(m_timeout_timer_id, APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER), NULL);
					APP_ERROR_CHECK(err_code);
					while(!ble_send_dex_pkt(&pkt) && !timeout) {nrf_delay_ms(500);};
					err_code = app_timer_stop(m_timeout_timer_id);
					APP_ERROR_CHECK(err_code);
				}
			}

			if(!needsTimingCalibration)
			{
				if(timeout || !got_packet)
				{
					//Indicate fault conditions
					//BLUE + RED = BLE error/timeout
					//RED = No dex pkt received
					LEDS_ON(LED_RGB_RED_MASK);
				}
				//We are done. Lets go to sleep...
				power_manage();
			}

    	}

    }



}

/**
 * @}
 */
