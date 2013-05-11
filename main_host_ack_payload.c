/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
*
* The information contained herein is property of Nordic Semiconductor ASA.
* Terms and conditions of usage are described in detail in NORDIC
* SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
*
* Licensees are granted free, non-transferable use of the information. NO
* WARRANTY of ANY KIND is provided. This heading must NOT be removed from
* the file.
*
* $LastChangedRevision: 15516 $
*/

/** 
 * @file
 * @brief Gazell Link Layer Host with Payload in ACK example
 * @defgroup gzll_host_ack_payload_example Gazell Link Layer Host with Payload in ACK
 * @{
 * @ingroup gzll_03_examples
 *
 * This project requires that a Device running the 
 * @ref gzll_device_ack_payload_example be used as a counterpart for 
 * receiving the data. This can be on either nRF51 device or a nRF24Lxx device
 * running the \b gzll_device_ack_payload example in the nRFgo SDK. 
 * 
 * This example listens for a packet and sends an ACK
 * when a packet is received. The contents of the first payload byte of 
 * the received packet is output on the GPIO Port BUTTONS. 
 * The contents of GPIO Port LEDS are sent in the first payload byte (byte 0) 
 * of the ACK packet.
 */


#include "nrf_gzll.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include <stdint.h>
#include <math.h>
#include "nrf.h"
#include "twi_master.h"
#include "app_timer.h"
#include "radio_config.h"
#include "libjanek.h"
#include "spi_master.h"
#include "wavetable.h"
#include "tune_mai.h"

/*****************************************************************************/
/** @name Configuration  */
/*****************************************************************************/

// Define pipe
#define PIPE_NUMBER 0 ///< We use pipe 0 in this example

// GPIO
#define BUTTONS NRF_GPIO_PORT_SELECT_PORT0 ///< GPIO port for reading from buttons
#define LEDS    NRF_GPIO_PORT_SELECT_PORT1 ///< GPIO port for writing to LEDs

// Define payload length
#define TX_PAYLOAD_LENGTH 1 ///< We use 1 byte payload length when transmitting

// Data and acknowledgement payloads
static uint8_t data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];  ///< Placeholder for data payload received from host. 
static uint8_t ack_payload[TX_PAYLOAD_LENGTH];                   ///< Payload to attach to ACK sent to device.

// Debug helper variables
extern nrf_gzll_error_code_t nrf_gzll_error_code;   ///< Error code
static bool init_ok, enable_ok, push_ok, pop_ok, packet_received;  
/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/

#define VOLUME 32
#define OSCILLATOR_COUNT 5 // 16 oscillators: 25% load with -O1 (64: 90%)
#define TICKS_LIMIT 20 //203 // 39062 / ( 4 * 48 )
#define TIMER_CC (410 * 2)
#define PITCH_SHIFT 1

#undef NULL
#define NULL 0                   /* see <stddef.h> */

static int timeout_count = 0;
#define MAX_TIMEOUT_COUNT 100


#define MAX_TX_ATTEMPTS 200

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 5                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */
#define SLEEP_SAMPLE_INTERVAL                APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
static app_timer_id_t                        m_sample_timer_id;                        /**< Sampling timer. */



/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
 
    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}

/** @} */





#define TPA2011_EN_PIN 5

void TPA2011_init(void)
{
    nrf_gpio_pin_write(TPA2011_EN_PIN, 1);
	nrf_gpio_cfg_output(TPA2011_EN_PIN);
}


#define POT 8 // power of two; must match with scale_table values
#define ENVPOT 7
#define CLIP 32767 // 127 * 8

#define MAG3110_ADDR (0x0E << 1)
#define MAG3110_WHO_AM_I 0x07

static uint8_t mag_reg_who_am_i[] = { 0x07 };
static uint8_t mag_reg_data[] = { 0x01 };
static uint8_t mag_config[] = { 0x10, 0xC9, 0x80 };

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mag_data_t;


static void MAG_init(void)
{
	twi_master_transfer(MAG3110_ADDR, mag_config, 3, TWI_ISSUE_STOP);
}
static void MAG_read(mag_data_t* mag_data)
{
	uint8_t m_data[6];
    twi_master_transfer(MAG3110_ADDR, mag_reg_data, 1, TWI_DONT_ISSUE_STOP);
	twi_master_transfer(MAG3110_ADDR | TWI_READ_BIT, m_data, 6, TWI_ISSUE_STOP);
    mag_data->x = (m_data[0] << 8) | m_data[1];
    mag_data->y = (m_data[2] << 8) | m_data[3];
    mag_data->z = (m_data[4] << 8) | m_data[5];   
}

mag_data_t current_mag_data;
/**@brief Sample timer timeout handler.
 *
 * @details This function will be called each time the sample timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
void sample_timeout_handler(void * p_context)
{
    MAG_read(&current_mag_data);
}
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_sample_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sample_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
/**@brief Start sample timer, starting in sleep mode.
 */
static void sample_timer_start(void)
{
    uint32_t err_code;
    
    // Start application timers
    err_code = app_timer_start(m_sample_timer_id, SLEEP_SAMPLE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
uint32_t* spi_base_address;


static void play_song(const struct event_t * song, uint16_t event_count)
{
    uint32_t outval;
    uint8_t outbuf[3];
    uint8_t inbuf[3];
    uint8_t data[6];
    mag_data_t mag_data;
    int volume = VOLUME;
    volatile NRF_TIMER_Type * p_timer = NRF_TIMER2;
    p_timer->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Timer Mode
    p_timer->PRESCALER = 0;                 // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us
    p_timer->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // 16-bit mode

    p_timer->TASKS_CLEAR = 1;               // clear the task first to be usable for later
    p_timer->CC[0] = TIMER_CC;

    uint16_t increments_pot[ OSCILLATOR_COUNT ];
    uint32_t phase_accu_pot[ OSCILLATOR_COUNT ];
    uint32_t envelope_positions_envpot[ OSCILLATOR_COUNT ];
    for ( uint8_t osc = 0; osc < OSCILLATOR_COUNT; ++osc ) {
        increments_pot[ osc ] = 0;
        phase_accu_pot[ osc ] = 0;
        envelope_positions_envpot[ osc ] = 0;
    }
    uint8_t next_osc = 0;
    uint16_t ticks = 0;
    uint32_t time = 0;
    uint16_t event_index = 0;
    const uint32_t sizeof_wt_pot = ( (uint32_t)sizeof( wt ) << POT );
    const uint32_t sizeof_wt_sustain_pot = ( (uint32_t)sizeof( wt_sustain ) << POT );
    const uint32_t sizeof_wt_attack_pot = ( (uint32_t)sizeof( wt_attack ) << POT );
    const uint32_t sizeof_envelope_table_envpot = ( (uint32_t)sizeof( envelope_table ) << ENVPOT );
    time = song[ event_index ].time;
    
    p_timer->TASKS_START = 1;               // Start clocks
    while ( true ) {
        //LATBbits.LATB14 = 1; // load visualization pin
        if ( time >= song[ event_index ].time ) {
            increments_pot[ next_osc ] = scale_table[ song[ event_index ].pitch ] << PITCH_SHIFT;
            phase_accu_pot[ next_osc ] = 0;
            envelope_positions_envpot[ next_osc ] = 0;
            ++next_osc;
            if ( next_osc >= OSCILLATOR_COUNT ) {
                next_osc = 0;
            }
            ++event_index;
            if ( event_index >= event_count ) {
                ticks = 0;
                time = 0;
                event_index = 0;
            }
        }
        ticks += 1;
        if ( ticks >= TICKS_LIMIT ) {
            ticks = 0;
            time += 1;
        }

        int32_t value = 0;
        for ( uint8_t osc = 0; osc < OSCILLATOR_COUNT; ++osc ) {
            phase_accu_pot[ osc ] += (increments_pot[ osc ]);
            if ( phase_accu_pot[ osc ] >= sizeof_wt_pot ) {
                phase_accu_pot[ osc ] -= sizeof_wt_sustain_pot;
            }
            uint16_t phase_accu = ( phase_accu_pot[ osc ] >> POT );
            value += envelope_table[ envelope_positions_envpot[ osc ] >> ENVPOT ] * wt[ phase_accu ] * VOLUME;
            if ( phase_accu_pot[ osc ] >= sizeof_wt_attack_pot &&
                 envelope_positions_envpot[ osc ] < sizeof_envelope_table_envpot - 1 )
            {
                ++envelope_positions_envpot[ osc ];
            }
        }
        value >>= 8; // envelope_table resolution
        /*
        if ( value > CLIP ) {
            LED_set(1);
            value = CLIP;
        } else if ( value < -CLIP ) {
            LED_set(1);
            value = -CLIP;
        }
        */
        //LATBbits.LATB14 = 0; // load visualization pin
        //while ( TMR2 > 10 ) {
        //}
        if (nrf_gpio_pin_read(1) == 0) {
            while((nrf_gpio_pin_read(1) == 0));
            nrf_gpio_pin_write(TPA2011_EN_PIN, 0);
            for (int j = 0; j < 10000; j++) {
                __nop();
            }
            NRF_POWER->SYSTEMOFF = 1;
        }
        if (p_timer->EVENTS_COMPARE[0] != 0) {
            LED_set(1);
        }
        while (p_timer->EVENTS_COMPARE[0] == 0)
        {
        }
        p_timer->EVENTS_COMPARE[0] = 0;
        p_timer->TASKS_CLEAR = 1;
        outval =  (32767 + (value)) << 6;
        outbuf[0] = (outval >> 16) & 0x3F;
        outbuf[1] = (outval >> 8) & 0xFF;
        outbuf[2] = (outval) & 0xFF;
        spi_master_tx_rx(spi_base_address, 3, outbuf, inbuf);
    }
}

/*****************************************************************************/
/** 
* @brief Main function. 
* 
* @return ANSI required int return type.
*/
/*****************************************************************************/
//lint -save -e514 Unusual use of a boolean expression (use of &= assignment).

int main2()
{
    uint32_t outval;
    uint8_t outbuf[3];
    uint8_t inbuf[3];
    uint8_t data[6];
    mag_data_t mag_data;
    int volume = VOLUME;
    NRF_CLOCK->LFCLKSRC = 0; // RC Timer
    
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) 
    {
    }
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
    }
    GPIO_WAKEUP_BUTTON_WITH_PULLUP_CONFIG(1);
    if (nrf_gpio_pin_read(1) == 0) {
    while((nrf_gpio_pin_read(1) == 0));
    //nrf_gpio_pin_write(TPA2011_EN_PIN, 0);
    for (int j = 0; j < 10000; j++) {
        __nop();
    }
    }


	twi_master_init();
	twi_master_transfer(MAG3110_ADDR, mag_reg_who_am_i, 1, TWI_DONT_ISSUE_STOP);
	twi_master_transfer(MAG3110_ADDR | TWI_READ_BIT, data, 1, TWI_ISSUE_STOP);

	twi_master_transfer(MAG3110_ADDR, mag_config, 3, TWI_ISSUE_STOP);

	MAG_init();
    
   
	MMA_init();
    TMP_getdata(data);
	LED_init();
    
    
    CHRG_init();
    TPA2011_init();
    spi_base_address = spi_master_init(SPI0, SPI_MODE0, 0);
    const uint16_t event_count = sizeof( tune_mai ) / sizeof( tune_mai[ 0 ] );
    play_song(tune_mai, event_count);
    timers_init();
    sample_timer_start();
        
	//LED_set(1);
	
	
    // Initialize Gazell
    init_ok = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    
    // Attempt sending every packet up to 100 times    
    init_ok &= nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);

    nrf_gzll_set_timeslot_period(3000);
    nrf_gzll_set_base_address_0(BASE_ADDRESS);
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_4_DBM);
    nrf_gzll_set_datarate(NRF_GZLL_DATARATE_1MBIT);
    nrf_gzll_set_xosc_ctl(NRF_GZLL_XOSC_CTL_AUTO);
    // Enable Gazell to start sending over the air
    enable_ok = nrf_gzll_enable();         


    while(1)
    {
        // Error handling

        // Optionally send the CPU to sleep while waiting for a callback.
        __WFI();
    }
}

/** @} */

/*****************************************************************************/
/**
* @brief Main function.
* @return ANSI required int return type.
*/
/*****************************************************************************/
int main()
{
    uint8_t debug_led_output;

    // Setup port directions
    nrf_gpio_cfg_output(0);
    nrf_gpio_pin_clear(0);
    
    // Initialize Gazell
    init_ok = nrf_gzll_init(NRF_GZLL_MODE_HOST);  
    
    // Load data into TX queue
    //ack_payload[0] = nrf_gpio_port_read(BUTTONS);  // Button logic is inverted.
    ack_payload[0] = 0;
    push_ok = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);

    //nrf_gzll_enable_rssi();   
    // Enable Gazell to start sending over the air
    enable_ok = nrf_gzll_enable();

    while(1)
    {

        // Optionally send the CPU to sleep while waiting for a callback.
        __WFI();
        nrf_gpio_pin_clear(0);
    }
}


/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/


// If a data packet was received, we write the first byte to LEDS. 
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{   
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    packet_received = true;

    // Pop packet and write first byte of the payload to the GPIO port.
    pop_ok = nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload, &data_payload_length);
    if ((data_payload_length > 0) && (rx_info.rssi > -55))
    {
        nrf_gpio_pin_set(0);
    }

    // Read buttons and load ACK payload into TX queue
    ack_payload[0] = nrf_gpio_port_read(BUTTONS);  // Button logic is inverted.
    push_ok = nrf_gzll_add_packet_to_tx_fifo(pipe, ack_payload, TX_PAYLOAD_LENGTH);

}

// Callbacks not needed in this example.
void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}

/** @} */
/** @} */

