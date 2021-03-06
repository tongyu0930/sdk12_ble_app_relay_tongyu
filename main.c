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

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_error.h"


#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//不知道设置为0004和4000有什么区别，led看起来都闪的一样快。
//#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_INTERVAL           0x0400   //Scan interval or window is between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s).
//#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0400   //The scanWindow shall be less than or equal to the scanInterval.Scan window between 0x0004 and 0x4000
#define SCAN_ACTIVE             0                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

//volatile uint8_t *ppp_data[31];
static bool button1_pressed = false;

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */


/**
 * @brief Parameters used when scanning.   active scan means asking for scan response packet
 */
static const ble_gap_scan_params_t m_scan_params =
  {
    .active      = SCAN_ACTIVE,
    .use_whitelist   = SCAN_SELECTIVE,
    .adv_dir_report = 0,
    .interval    = SCAN_INTERVAL,
    .window      = SCAN_WINDOW,
    .timeout     = SCAN_TIMEOUT
  };


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
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    uint8_t data[] = "xxxxx"; // Our data to adverise。 scanner上显示的0x串中，最后是00，表示结束。

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = data;
    manuf_specific_data.data.size   = sizeof(data);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(&m_adv_params); // 括号里面的是个全局变量吗？
    APP_ERROR_CHECK(err_code);
}


//自己加的
void relay_adv_data(ble_evt_t * p_ble_evt)
{
	uint32_t index = 0;
	//uint8_t pp_data[31] = {0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	// 这个data必须是［31］，不是的话sd_ble_gap_adv_data_set就不管用。
	//pp_data[0] = 0x1e;
	//pp_data[1] = 0xff;
	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

				if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
				{
					uint8_t c = field_length +1;
					uint8_t pp_data[c];
					uint8_t a = index+2;
					uint8_t b = 2;

					pp_data[0] = field_length; // 你要relay的数据的长度和你扫描到的一样长
					pp_data[1] = field_type;

					while(a <= (index+field_length))
					{
						pp_data[b] = p_data[a];
						NRF_LOG_INFO("p_data = %x\r\n", p_data[a]);
						a++;
						b++;
					}

					sd_ble_gap_adv_data_set(pp_data, sizeof(pp_data), NULL, 0);
					nrf_drv_gpiote_out_toggle(BSP_LED_3);
				}

	            index += field_length + 1;
	    }

	//sd_ble_gap_adv_data_set(p_data, sizeof(p_data), NULL, 0);

}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    relay_adv_data(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) //这个东西就是当设置的input被激发时，所唤起的function
                                                                            //这里的“action”有三种actions，是哪一种啊？
{
	if(!button1_pressed)
	{
    //sd_ble_gap_adv_stop(); // 如果去掉这句话，按下按键1，系统就会自动不断的切换广播和扫描。
    uint32_t err_code;
    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("OK\r\n");
    button1_pressed = true;
	}
}


static void gpio_config()
{
	ret_code_t err_code;

    // Initialze driver.
    err_code = nrf_drv_gpiote_init(); //Function for initializing the GPIOTE module.
    APP_ERROR_CHECK(err_code);

    // Configure output LED3
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(BSP_LED_3, &out_config);
    APP_ERROR_CHECK(err_code);
    // Set output pins (this will turn off the LED's). 这个function的意思是设置为低电平？
    nrf_drv_gpiote_out_set(BSP_LED_3);

    // Configure input Button1
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true); //对于in_config这个结构体里，high_accuracy=ture, is_watcher＝false，
                                                                                    //提升电阻默认关，sense = NRF_GPIOTE_POLARITY_TOGGLE（sense对应上面的action，这里设置input电位有变化时激发interrupt）
    in_config.pull = NRF_GPIO_PIN_PULLUP; //对于上面in_config这个结构体的其中一个元素的设置。Pin pullup resistor enabled。因为GPIOTE_CONFIG_IN_SENSE_TOGGLE()里面提升电阻默认是关闭的。

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, in_pin_handler); // 这个input被触发时就激发in_pin_handler, 其实就是说input被按下时就呼叫in_pin_handler
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_1, true); //Function for enabling sensing of a GPIOTE input pin.
    //true to enable the interrupt.
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    ble_stack_init();
    gpio_config();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("TONG_YU RELAY START\r\n");
    NRF_LOG_INFO("Press button 1 to start scan\r\n");
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
