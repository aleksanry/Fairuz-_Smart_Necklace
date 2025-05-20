/**
 * Fairuzé - Smart Emotion & Health Monitoring Necklace
 * Target Hardware: nRF52832/nRF52840
 * Written By Nermin Maddouri     
 * This firmware implements a wearable health and emotion monitoring necklace
 * with BLE connectivity and multiple embedded sensors 
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_twi.h"  // For I2C communication with sensors
#include "nrf_drv_spi.h"  // For SPI communication with sensors

// Include sensor drivers
#include "max30102.h"     // Heart rate sensor
#include "lis2dh12.h"     // Accelerometer
#include "si7021.h"       // Temperature sensor
#include "gsr_driver.h"   // Custom GSR sensor driver
#include "led_control.h"  // LED indicator control
#include "haptic.h"       // Vibration motor control

// Constants
#define DEVICE_NAME                     "Fairuzé"
#define MANUFACTURER_NAME               "Smart Jewelry Co"
#define MODEL_NUMBER                    "FRZ-001"
#define FIRMWARE_VERSION                "1.0.3"
#define HARDWARE_REVISION               "A"

#define APP_BLE_OBSERVER_PRIO           3
#define APP_BLE_CONN_CFG_TAG            1
#define APP_ADV_INTERVAL                300                                     // The advertising interval (in units of 0.625 ms)
#define APP_ADV_DURATION                18000                                   // The advertising duration (180 seconds) in units of 10 milliseconds

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        // Minimum acceptable connection interval
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        // Maximum acceptable connection interval
#define SLAVE_LATENCY                   0                                       // Slave latency
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         // Connection supervisory timeout

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(120000)                 // Battery level measurement interval (120 seconds)
#define HEALTH_MONITORING_INTERVAL      APP_TIMER_TICKS(5000)                   // Health monitoring interval (5 seconds)

#define CROWN_LED_PIN                   13                                      // Pin for crown LED
#define VIBRATION_MOTOR_PIN             14                                      // Pin for vibration motor
#define TOUCH_SENSOR_PIN                15                                      // Pin for touch sensor
#define BATTERY_VOLTAGE_PIN             NRF_SAADC_INPUT_AIN0                    // Pin for battery voltage monitoring

// TWI (I2C) and SPI configuration
#define TWI_SCL_PIN                     26                                      // SCL pin for I2C sensors
#define TWI_SDA_PIN                     27                                      // SDA pin for I2C sensors
#define SPI_SCK_PIN                     29                                      // SPI clock pin
#define SPI_MOSI_PIN                    30                                      // SPI MOSI pin
#define SPI_MISO_PIN                    31                                      // SPI MISO pin
#define SPI_CS_HR_PIN                   28                                      // SPI chip select for heart rate sensor

// Sensor I2C addresses
#define ACCEL_ADDR                      0x19                                    // LIS2DH12 accelerometer address
#define TEMP_ADDR                       0x40                                    // Si7021 temperature sensor address

// Service UUIDs
static ble_uuid_t m_adv_uuids[] =                                               // UUIDs used for advertising
{
    {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
};

// Custom emotional state service UUID
#define BLE_UUID_EMOTIONAL_STATE_SERVICE 0x1400                                 // Custom service UUID
#define BLE_UUID_EMOTIONAL_STATE_CHAR    0x1401                                 // Custom characteristic UUID

// Global variables
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        // Handle of the current connection
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);                          // I2C instance
static nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(0);                          // SPI instance
APP_TIMER_DEF(m_battery_timer_id);                                             // Battery timer
APP_TIMER_DEF(m_health_monitor_timer_id);                                      // Health monitoring timer

// BLE services
BLE_BAS_DEF(m_bas);                                                            // Battery service instance
BLE_HRS_DEF(m_hrs);                                                            // Heart rate service instance
BLE_HTS_DEF(m_hts);                                                            // Health thermometer service instance

// Device state
typedef struct {
    bool is_active;                     // Is the device active
    bool is_charging;                   // Is the device charging
    uint8_t battery_level;              // Current battery level (percent)
    uint8_t heart_rate;                 // Current heart rate (bpm)
    float skin_temperature;             // Current skin temperature (Celsius)
    uint16_t gsr_value;                 // Current GSR value
    int16_t accel_values[3];            // X, Y, Z acceleration values
    uint8_t ambient_light;              // Ambient light level
    uint8_t current_emotion;            // Current detected emotion (0=neutral, 1=happy, 2=calm, 3=stressed, 4=sad)
    uint8_t emotion_intensity;          // Intensity of current emotion (0-100)
} device_state_t;

static device_state_t m_device_state = {
    .is_active = false,
    .is_charging = false,
    .battery_level = 100,
    .heart_rate = 0,
    .skin_temperature = 0.0f,
    .gsr_value = 0,
    .accel_values = {0, 0, 0},
    .ambient_light = 0,
    .current_emotion = 0,
    .emotion_intensity = 0
};

// Function declarations
static void initialize_sensors(void);
static void read_sensors(void);
static void analyze_emotion(void);
static void analyze_health(void);
static void check_alert_conditions(void);
static void send_notification(const char* title, const char* message);
static void advertising_start(void);
static void sleep_mode_enter(void);

/**
 * @brief Function for initializing the application
 */
static void app_initialize(void)
{
    // Initialize logging
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("Fairuzé necklace initializing...");
    
    // Initialize power management
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    
    // Initialize GPIOTE for buttons and LEDs
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    
    // Setup crown LED
    nrf_drv_gpiote_out_config_t led_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(CROWN_LED_PIN, &led_config);
    APP_ERROR_CHECK(err_code);
    
    // Setup vibration motor
    nrf_drv_gpiote_out_config_t motor_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(VIBRATION_MOTOR_PIN, &motor_config);
    APP_ERROR_CHECK(err_code);
    
    // Setup touch sensor input
    nrf_drv_gpiote_in_config_t touch_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    err_code = nrf_drv_gpiote_in_init(TOUCH_SENSOR_PIN, &touch_config, touch_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(TOUCH_SENSOR_PIN, true);
    
    // Initialize I2C (TWI)
    nrf_drv_twi_config_t twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    twi_config.scl = TWI_SCL_PIN;
    twi_config.sda = TWI_SDA_PIN;
    twi_config.frequency = NRF_DRV_TWI_FREQ_400K;
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    
    // Initialize SPI
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.sck_pin = SPI_SCK_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.ss_pin = SPI_CS_HR_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
    err_code = nrf_drv_spi_init(&m_spi, &spi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    // Initialize SAADC for battery monitoring
    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
    
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_VOLTAGE_PIN);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
    
    // Initialize sensors
    initialize_sensors();
    
    // Initialize device state
    m_device_state.is_active = true;
    NRF_LOG_INFO("Fairuzé necklace initialized successfully");
    
    // Short vibration to indicate successful initialization
    nrf_drv_gpiote_out_set(VIBRATION_MOTOR_PIN);
    nrf_delay_ms(200);
    nrf_drv_gpiote_out_clear(VIBRATION_MOTOR_PIN);
}

/**
 * @brief Function to initialize all sensors
 */
static void initialize_sensors(void)
{
    ret_code_t err_code;
    
    NRF_LOG_INFO("Initializing sensors...");
    
    // Initialize accelerometer (LIS2DH12)
    lis2dh12_init(&m_twi, ACCEL_ADDR);
    
    // Initialize temperature sensor (Si7021)
    si7021_init(&m_twi, TEMP_ADDR);
    
    // Initialize heart rate sensor (MAX30102)
    max30102_init(&m_spi);
    
    // Initialize GSR sensor (custom analog sensor)
    gsr_driver_init();
    
    NRF_LOG_INFO("Sensors initialized");
}

/**
 * @brief Function to read data from all sensors
 */
static void read_sensors(void)
{
    // Read heart rate
    m_device_state.heart_rate = max30102_read_heart_rate();
    
    // Read skin temperature
    m_device_state.skin_temperature = si7021_read_temperature();
    
    // Read accelerometer
    lis2dh12_read_xyz(m_device_state.accel_values);
    
    // Read GSR value
    m_device_state.gsr_value = gsr_driver_read();
    
    // Update heart rate service
    ble_hrs_heart_rate_measurement_send(&m_hrs, m_device_state.heart_rate);
    
    // Update temperature service (convert to proper format for BLE HTS)
    float temp_in_celsius = m_device_state.skin_temperature;
    int32_t temp_for_ble = (int32_t)(temp_in_celsius * 100);
    ble_hts_measurement_t hts_meas;
    hts_meas.temp_in_fahr_units = false;
    hts_meas.temp_type_present = true;
    hts_meas.temp_type = BLE_HTS_TEMP_TYPE_BODY;
    hts_meas.temp = temp_for_ble;
    ble_hts_measurement_send(&m_hts, &hts_meas);
    
    NRF_LOG_INFO("Sensor readings - HR: %d, Temp: %d.%02d, GSR: %d", 
                 m_device_state.heart_rate,
                 (int)m_device_state.skin_temperature, 
                 (int)(m_device_state.skin_temperature * 100) % 100,
                 m_device_state.gsr_value);
}

/**
 * @brief Function to analyze emotions based on sensor data
 */
static void analyze_emotion(void)
{
    // Simple emotion analysis algorithm based on HR and GSR
    // In a real product, this would be much more sophisticated
    
    uint8_t prev_emotion = m_device_state.current_emotion;
    
    // Check for stressed state
    if (m_device_state.heart_rate > 90 && m_device_state.gsr_value > 400) {
        m_device_state.current_emotion = 3; // Stressed
        m_device_state.emotion_intensity = MIN(100, (m_device_state.heart_rate - 90) + (m_device_state.gsr_value - 400) / 10);
    }
    // Check for calm state
    else if (m_device_state.heart_rate < 70 && m_device_state.gsr_value < 300) {
        m_device_state.current_emotion = 2; // Calm
        m_device_state.emotion_intensity = MIN(100, (70 - m_device_state.heart_rate) + (300 - m_device_state.gsr_value) / 10);
    }
    // Check for happy state (based on activity and moderate GSR)
    else if (m_device_state.heart_rate > 75 && m_device_state.heart_rate < 90 && 
             m_device_state.gsr_value > 250 && m_device_state.gsr_value < 400) {
        m_device_state.current_emotion = 1; // Happy
        m_device_state.emotion_intensity = 50 + (m_device_state.heart_rate - 75) * 2;
    }
    // Default neutral state
    else {
        m_device_state.current_emotion = 0; // Neutral
        m_device_state.emotion_intensity = 30;
    }
    
    // Update emotional state service if emotion changed
    if (prev_emotion != m_device_state.current_emotion) {
        uint8_t emotion_data[2] = {m_device_state.current_emotion, m_device_state.emotion_intensity};
        // This would send the updated emotion value to the connected device
        // ble_emotional_state_update(&m_emotional_state_service, emotion_data, sizeof(emotion_data));
        
        NRF_LOG_INFO("Emotion changed to: %d, intensity: %d", 
                     m_device_state.current_emotion, m_device_state.emotion_intensity);
                     
        // Change crown LED color based on emotion
        switch(m_device_state.current_emotion) {
            case 1: // Happy - Yellow
                led_control_set_color(255, 255, 0); 
                break;
            case 2: // Calm - Blue
                led_control_set_color(0, 0, 255);
                break;
            case 3: // Stressed - Red
                led_control_set_color(255, 0, 0);
                break;
            default: // Neutral - White
                led_control_set_color(255, 255, 255);
        }
    }
}

/**
 * @brief Function to analyze health metrics
 */
static void analyze_health(void)
{
    // In a real implementation, this would include more sophisticated analysis
    // Check for abnormal heart rate
    if (m_device_state.heart_rate > 120 || m_device_state.heart_rate < 45) {
        NRF_LOG_WARNING("Abnormal heart rate detected: %d", m_device_state.heart_rate);
        // Could trigger an alert here
    }
    
    // Check for abnormal temperature
    if (m_device_state.skin_temperature > 38.0 || m_device_state.skin_temperature < 35.0) {
        NRF_LOG_WARNING("Abnormal skin temperature detected: %d.%02d", 
                        (int)m_device_state.skin_temperature, 
                        (int)(m_device_state.skin_temperature * 100) % 100);
        // Could trigger an alert here
    }
}

/**
 * @brief Function to check if any alert conditions are met
 */
static void check_alert_conditions(void)
{
    // Check for high stress alert
    if (m_device_state.current_emotion == 3 && m_device_state.emotion_intensity > 80) {
        send_notification("High Stress Detected", "Taking deep breaths may help reduce your stress levels.");
        
        // Haptic feedback - gentle pulsing vibration
        for (int i = 0; i < 3; i++) {
            nrf_drv_gpiote_out_set(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(200);
            nrf_drv_gpiote_out_clear(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(300);
        }
    }
    
    // Check for extended period of low activity
    // This would require additional tracking of activity over time
    
    // Low battery alert
    if (m_device_state.battery_level < 15 && !m_device_state.is_charging) {
        static uint32_t last_battery_alert_time = 0;
        uint32_t current_time = app_timer_cnt_get();
        
        // Send battery alert maximum once every 30 minutes
        if (last_battery_alert_time == 0 || 
            app_timer_cnt_diff_compute(current_time, last_battery_alert_time) > APP_TIMER_TICKS(30 * 60 * 1000)) {
            
            send_notification("Low Battery", "Please charge your Fairuzé necklace soon.");
            last_battery_alert_time = current_time;
            
            // Short double vibration for battery alert
            nrf_drv_gpiote_out_set(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(100);
            nrf_drv_gpiote_out_clear(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(100);
            nrf_drv_gpiote_out_set(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(100);
            nrf_drv_gpiote_out_clear(VIBRATION_MOTOR_PIN);
        }
    }
}

/**
 * @brief Timer handler for health monitoring
 */
static void health_monitoring_timer_handler(void * p_context)
{
    if (m_device_state.is_active) {
        read_sensors();
        analyze_emotion();
        analyze_health();
        check_alert_conditions();
    }
}

/**
 * @brief Timer handler for battery level measurement
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    if (m_device_state.is_active) {
        // Read battery voltage using SAADC
        // Convert to battery percentage
        // In a real implementation, this would read actual battery voltage
        
        if (!m_device_state.is_charging) {
            // Simulate battery drain when not charging
            if (m_device_state.battery_level > 0) {
                m_device_state.battery_level -= 1;
            }
        }
        else {
            // Simulate charging
            if (m_device_state.battery_level < 100) {
                m_device_state.battery_level += 5;
                if (m_device_state.battery_level > 100) {
                    m_device_state.battery_level = 100;
                }
            }
        }
        
        ret_code_t err_code = ble_bas_battery_level_update(&m_bas, m_device_state.battery_level, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

/**
 * @brief Function for handling BLE events
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
            
        default:
            // No implementation needed
            break;
    }
}

/**
 * @brief Function for initializing BLE stack
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    // Configure the BLE stack using the default settings
    // Fetch the start address of the application RAM
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Enable BLE stack
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Register a handler for BLE events
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**
 * @brief Function for initializing the GAP
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for initializing services
 */
static void services_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init;
    ble_hrs_init_t hrs_init;
    ble_hts_init_t hts_init;
    
    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialize Heart Rate Service
    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = NULL;

    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));
    
    hts_init.evt_handler                 = NULL;
    hts_init.temp_type_as_characteristic = true;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;
    
    hts_init.temp_cccd_wr_sec = SEC_OPEN;
    hts_init.temp_type_rd_sec = SEC_OPEN;
    
    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);
    
    // Here we would also initialize our custom Emotional State service
    // This code is omitted for brevity
}

/**
 * @brief Function for starting advertising
 */
static void advertising_start(void)
{
    ret_code_t           err_code;
    ble_advertising_t    advertising;
    ble_advdata_t        advdata;
    ble_advdata_t        srdata;
    ble_adv_modes_config_t config;
    
    memset(&advdata, 0, sizeof(advdata));
    memset(&srdata, 0, sizeof(srdata));
    
    // Set the advertisement data
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;
    
    // Configure advertisement settings
    memset(&config, 0, sizeof(config));
    
    config.ble_adv_fast_enabled  = true;
    config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    
    err_code = ble_advertising_init(&advertising, &advdata, &srdata, &config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = ble_advertising_start(&advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Advertising started");
}

/**
 * @brief Function for initializing timers
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    // Create timers
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_health_monitor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                health_monitoring_timer_handler);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for starting timers
 */
static void timers_start(void)
{
    ret_code_t err_code;
    
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_health_monitor_timer_id, HEALTH_MONITORING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for sending notifications to connected app
 */
static void send_notification(const char* title, const char* message)
{
    // In a real implementation, this would create a notification to be sent to the connected app
    NRF_LOG_INFO("Notification: %s - %s", title, message);
    
    // This is a placeholder for actual BLE notification mechanism
    // In a real implementation, you would create a custom BLE characteristic for notifications
    // and send the data through it
}

/**
 * @brief Function for handling touch sensor events
 */
static void touch_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == TOUCH_SENSOR_PIN) {
        static uint32_t last_touch_time = 0;
        uint32_t current_time = app_timer_cnt_get();
        
        // Debounce touch events
        if (last_touch_time == 0 || 
            app_timer_cnt_diff_compute(current_time, last_touch_time) > APP_TIMER_TICKS(500)) {
            
            last_touch_time = current_time;
            
            // Toggle crown LED on/off
            static bool led_on = false;
            led_on = !led_on;
            
            if (led_on) {
                // Turn on LED with color according to current emotion
                switch(m_device_state.current_emotion) {
                    case 1: // Happy - Yellow
                        led_control_set_color(255, 255, 0); 
                        break;
                    case 2: // Calm - Blue
                        led_control_set_color(0, 0, 255);
                        break;
                    case 3: // Stressed - Red
                        led_control_set_color(255, 0, 0);
                        break;
                    default: // Neutral - White
                        led_control_set_color(255, 255, 255);
                }
            } else {
                // Turn off LED
                led_control_set_color(0, 0, 0);
            }
            
            // Short vibration feedback
            nrf_drv_gpiote_out_set(VIBRATION_MOTOR_PIN);
            nrf_delay_ms(50);
            nrf_drv_gpiote_out_clear(VIBRATION_MOTOR_PIN);
            
            NRF_LOG_INFO("Touch detected, LED toggled to %s", led_on ? "ON" : "OFF");
        }
    }
}

/**
 * @brief Function for handling SAADC events
 */
static void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        // Battery voltage measurement complete
        // This would convert ADC reading to actual battery voltage and percentage
        // Omitted for brevity
    }
}

/**
 * @brief Function for entering device sleep mode
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;
    
    // Turn off LEDs
    led_control_set_color(0, 0, 0);
    
    // Stop timers
    err_code = app_timer_stop(m_health_monitor_timer_id);
    APP_ERROR_CHECK(err_code);
    
    // Keep battery monitoring active at lower frequency
    
    // Put sensors into low power mode
    max30102_sleep();
    lis2dh12_sleep();
    
    // Update device state
    m_device_state.is_active = false;
    
    NRF_LOG_INFO("Entering sleep mode");
    
    // Wait for an event
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for waking up the device
 */
static void wake_up(void)
{
    ret_code_t err_code;
    
    if (!m_device_state.is_active) {
        // Wake up sensors
        max30102_wakeup();
        lis2dh12_wakeup();
        
        // Restart health monitoring timer
        err_code = app_timer_start(m_health_monitor_timer_id, HEALTH_MONITORING_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
        
        // Update device state
        m_device_state.is_active = true;
        
        // Flash LED to indicate wake up
        led_control_set_color(255, 255, 255);
        nrf_delay_ms(200);
        led_control_set_color(0, 0, 0);
        
        NRF_LOG_INFO("Device woken up");
    }
}

/**
 * @brief Function for handling idle state
 */
static void idle_state_handle(void)
{
    // If no activity for a period, enter sleep mode
    static uint32_t last_activity_time = 0;
    static bool idle_state_entered = false;
    
    uint32_t current_time = app_timer_cnt_get();
    
    // Check for device inactivity (15 minutes)
    if (m_device_state.is_active && 
        app_timer_cnt_diff_compute(current_time, last_activity_time) > APP_TIMER_TICKS(15 * 60 * 1000)) {
        
        if (!idle_state_entered) {
            idle_state_entered = true;
            sleep_mode_enter();
        }
    }
    else {
        idle_state_entered = false;
    }
    
    // Feed the watchdog if used
    
    // Process any pending system events
    nrf_pwr_mgmt_run();
}

/**
 * @brief Main application entry function
 */
int main(void)
{
    // Initialize
    app_initialize();
    timers_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_start();
    timers_start();
    
    NRF_LOG_INFO("Fairuzé necklace started");
    
    // Enter main loop
    while (true) {
        // Process logs
        NRF_LOG_PROCESS();
        
        // Handle system events and idle state
        idle_state_handle();
    }
}

// Implementation of LED control functions
void led_control_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    // In a real implementation, this would control RGB LEDs
    // For simplicity, we just control the crown LED pin
    if (r > 0 || g > 0 || b > 0) {
        nrf_drv_gpiote_out_set(CROWN_LED_PIN);
    }
    else {
        nrf_drv_gpiote_out_clear(CROWN_LED_PIN);
    }
    
    NRF_LOG_DEBUG("LED color set to R:%d G:%d B:%d", r, g, b);
}

// Implementation of MAX30102 heart rate sensor driver
void max30102_init(nrf_drv_spi_t const * p_spi)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    // Reset the sensor
    tx_data[0] = 0x09; // Mode configuration register
    tx_data[1] = 0x40; // Reset bit
    nrf_drv_spi_transfer(p_spi, tx_data, 2, rx_data, 2);
    nrf_delay_ms(10);
    
    // Configure sensor for heart rate mode
    tx_data[0] = 0x09; // Mode configuration register
    tx_data[1] = 0x03; // Heart rate mode
    nrf_drv_spi_transfer(p_spi, tx_data, 2, rx_data, 2);
    
    // Set sample rate and pulse width
    tx_data[0] = 0x0A; // SPO2 configuration register
    tx_data[1] = 0x27; // Sample rate 100Hz, pulse width 411us
    nrf_drv_spi_transfer(p_spi, tx_data, 2, rx_data, 2);
    
    // Set LED currents
    tx_data[0] = 0x0C; // LED1 pulse amplitude (RED)
    tx_data[1] = 0x1F; // 6.4mA
    nrf_drv_spi_transfer(p_spi, tx_data, 2, rx_data, 2);
    
    tx_data[0] = 0x0D; // LED2 pulse amplitude (IR)
    tx_data[1] = 0x1F; // 6.4mA
    nrf_drv_spi_transfer(p_spi, tx_data, 2, rx_data, 2);
    
    NRF_LOG_INFO("MAX30102 initialized");
}

uint8_t max30102_read_heart_rate(void)
{
    // In a real implementation, this would read from the sensor and calculate actual heart rate
    // For simplicity, we return simulated values
    static uint8_t simulated_hr = 70;
    static int8_t trend = 1;
    
    // Simulate slight variations in heart rate
    simulated_hr += trend;
    
    if (simulated_hr > 85) {
        trend = -1;
    }
    else if (simulated_hr < 65) {
        trend = 1;
    }
    
    return simulated_hr;
}

void max30102_sleep(void)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    // Put the sensor into shutdown mode
    tx_data[0] = 0x09; // Mode configuration register
    tx_data[1] = 0x80; // Shutdown mode
    nrf_drv_spi_transfer(&m_spi, tx_data, 2, rx_data, 2);
    
    NRF_LOG_DEBUG("MAX30102 put to sleep");
}

void max30102_wakeup(void)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    // Wake up the sensor and put it into heart rate mode
    tx_data[0] = 0x09; // Mode configuration register
    tx_data[1] = 0x03; // Heart rate mode
    nrf_drv_spi_transfer(&m_spi, tx_data, 2, rx_data, 2);
    
    NRF_LOG_DEBUG("MAX30102 woken up");
}

// Implementation of LIS2DH12 accelerometer driver
void lis2dh12_init(nrf_drv_twi_t const * p_twi, uint8_t addr)
{
    ret_code_t err_code;
    uint8_t reg[2];
    
    // Configure accelerometer for normal mode, 10Hz, XYZ enabled
    reg[0] = 0x20; // CTRL_REG1
    reg[1] = 0x27; // 10Hz, normal mode, XYZ enabled
    err_code = nrf_drv_twi_tx(p_twi, addr, reg, 2, false);
    APP_ERROR_CHECK(err_code);
    
    // Set scale to ±2g
    reg[0] = 0x23; // CTRL_REG4
    reg[1] = 0x00; // ±2g scale
    err_code = nrf_drv_twi_tx(p_twi, addr, reg, 2, false);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("LIS2DH12 initialized");
}

void lis2dh12_read_xyz(int16_t * p_values)
{
    ret_code_t err_code;
    uint8_t reg = 0x28 | 0x80; // OUT_X_L with auto-increment
    uint8_t data[6];
    
    // Read 6 bytes (X, Y, Z data)
    err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_twi_rx(&m_twi, ACCEL_ADDR, data, 6);
    APP_ERROR_CHECK(err_code);
    
    // Convert raw data to int16_t values
    p_values[0] = (data[1] << 8) | data[0]; // X
    p_values[1] = (data[3] << 8) | data[2]; // Y
    p_values[2] = (data[5] << 8) | data[4]; // Z
}

void lis2dh12_sleep(void)
{
    ret_code_t err_code;
    uint8_t reg[2];
    
    // Put accelerometer into power-down mode
    reg[0] = 0x20; // CTRL_REG1
    reg[1] = 0x00; // Power-down mode
    err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, reg, 2, false);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_DEBUG("LIS2DH12 put to sleep");
}

void lis2dh12_wakeup(void)
{
    ret_code_t err_code;
    uint8_t reg[2];
    
    // Put accelerometer back into normal mode
    reg[0] = 0x20; // CTRL_REG1
    reg[1] = 0x27; // 10Hz, normal mode, XYZ enabled
    err_code = nrf_drv_twi_tx(&m_twi, ACCEL_ADDR, reg, 2, false);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_DEBUG("LIS2DH12 woken up");
}

// Implementation of Si7021 temperature sensor driver
void si7021_init(nrf_drv_twi_t const * p_twi, uint8_t addr)
{
    ret_code_t err_code;
    uint8_t cmd = 0xFE; // Reset command
    
    // Reset the sensor
    err_code = nrf_drv_twi_tx(p_twi, addr, &cmd, 1, false);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(50); // Wait for reset to complete
    
    NRF_LOG_INFO("Si7021 initialized");
}

float si7021_read_temperature(void)
{
    ret_code_t err_code;
    uint8_t cmd = 0xF3; // Measure temperature command
    uint8_t data[2];
    
    // Send measure temperature command
    err_code = nrf_drv_twi_tx(&m_twi, TEMP_ADDR, &cmd, 1, false);
    APP_ERROR_CHECK(err_code);
    
    // Wait for measurement to complete
    nrf_delay_ms(20);
    
    // Read temperature data
    err_code = nrf_drv_twi_rx(&m_twi, TEMP_ADDR, data, 2);
    APP_ERROR_CHECK(err_code);
    
    // Convert raw data to temperature (formula from datasheet)
    uint16_t raw_temp = (data[0] << 8) | data[1];
    float temperature = ((175.72 * raw_temp) / 65536.0) - 46.85;
    
    // Simulate skin temperature from ambient temperature
    // In a real implementation, this would be actual skin temperature
    float skin_temp = temperature + 10.0; // ~10 degrees higher than ambient
    
    return skin_temp;
}

// Implementation of custom GSR sensor driver
void gsr_driver_init(void)
{
    // Setup ADC for GSR readings (if not already done in SAADC initialization)
    NRF_LOG_INFO("GSR sensor initialized");
}

uint16_t gsr_driver_read(void)
{
    // In a real implementation, this would read from the ADC connected to the GSR sensor
    // For simplicity, we return simulated values
    static uint16_t simulated_gsr = 300;
    static int16_t trend = 5;
    
    // Simulate slight variations in GSR
    simulated_gsr += trend;
    
    if (simulated_gsr > 450) {
        trend = -5;
    }
    else if (simulated_gsr < 250) {
        trend = 5;
    }
    
    return simulated_gsr;
}