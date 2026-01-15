/*
 * main.c - GPS and Environmental Monitoring System
 *
 * Created on: 2025 Feb 25 14:40:38
 * Author: Group X5 - Tiago Cunha 1161873, Rui Monteiro 1222171
 */

#include "DAVE.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/

/* Time structure for system time tracking */
typedef struct {
    int day;
    int hour;
    int minute;
    int seconds;
} Time_t;

/* GPS data structure with floating point values */
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    uint8_t satellites;
    uint8_t fix_quality;
    char time_utc[11];      // HHMMSS.SSS format
    char date[7];           // DDMMYY format
    float speed_knots;
    float course;
    uint8_t valid_data;
    char location[64];      // City/region name
    uint8_t location_found;
} GPS_Data_t;

/* GPS data structure with integer values for precision */
typedef struct {
    int32_t latitude_int;      // Latitude * 1000000
    int32_t longitude_int;     // Longitude * 1000000
    int32_t altitude_int;      // Altitude * 10
    uint8_t satellites;
    uint8_t fix_quality;
    char time_utc[11];
    char date[7];
    int32_t speed_knots_int;   // Speed * 1000
    uint8_t valid_data;
    char location[64];
    uint8_t location_found;
} GPS_Data_Int_t;

/******************************************************************************
 * DEFINES AND CONSTANTS
 ******************************************************************************/

/* AHT10 Temperature/Humidity Sensor */
#define AHT10_I2C_ADDR            (0x38 << 1)
#define AHT10_CMD_INIT            0xE1
#define AHT10_CMD_MEASURE         0xAC
#define AHT10_CMD_RESET           0xBA
#define AHT10_STATUS_BUSY         0x80
#define AHT10_STATUS_CALIBRATED   0x08

/* Buffer Sizes */
#define I2C_TX_BUFFER_SIZE        3
#define I2C_RX_BUFFER_SIZE        6
#define GPS_BUFFER_SIZE           256
#define NMEA_MAX_FIELDS          20
#define UART_BUFFER_SIZE         64

/* UART Commands */
#define CMD_GET_TEMPERATURE       "GET_TEMP"
#define CMD_GET_HUMIDITY          "GET_HUM"

/* Initial Message */
#define initial_message "Group X5, Tiago Cunha 1161873, Rui Monteiro 1222171\r\n"

/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* LED Control Variables */
volatile uint8_t led_mode = 0;              // 0=ON, 1=Blinking
volatile uint32_t interrupt_count = 0;       // Interrupt counter for LED timing
volatile uint32_t tempo = 50;               // LED blinking period in ms

/* System Time */
volatile Time_t system_time = {0, 0, 0, 0};

/* ADC Variables */
volatile int result = 0;                    // ADC result value
volatile uint8_t adc_flag = 0;              // ADC measurement flag

/* AHT10 Sensor Variables */
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];
static uint8_t i2c_rx_buffer[I2C_RX_BUFFER_SIZE];
static float last_temperature = 0.0f;
static float last_humidity = 0.0f;
static uint8_t sensor_initialized = 0;

/* GPS Variables */
static char gps_buffer[GPS_BUFFER_SIZE];
static uint16_t gps_index = 0;
static GPS_Data_t gps_data = {0};
static GPS_Data_t gps_stored_data = {0};
static GPS_Data_Int_t gps_data_int = {0};
static GPS_Data_Int_t gps_stored_data_int = {0};
static uint8_t gps_processing_enabled = 0;
static uint8_t gps_data_ready = 0;

/* UART Variables */
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t uart_rx_index = 0;
uint8_t uart_command_ready = 0;
char uart_tx_buffer[512];

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/* System Functions */
void send_initial_message(void);
void interrupcao_periodica(void);
void Button(void);
void AdcMeasurementHandler(void);

/* UART Functions */
void check_uart_data(void);
void process_uart_command(void);

/* AHT10 Sensor Functions */
void AHT10_Reset(void);
uint8_t AHT10_Init(void);
uint8_t AHT10_ReadValues(float *temperature, float *humidity);

/* GPS Functions */
void GPS_Init(void);
void GPS_ReadData(void);
void GPS_ProcessData(void);
GPS_Data_t* GPS_GetData(void);
void GPS_GetLocationFromCoordinates_Int(int32_t lat_int, int32_t lon_int, char* location);
void nmea_to_int(const char* nmea_str, int is_longitude, int32_t* result);
int simple_atoi(const char* str);
void parse_altitude_int(const char* str, int32_t* altitude);

/******************************************************************************
 * SYSTEM FUNCTIONS IMPLEMENTATION
 ******************************************************************************/

/**
 * Send initial identification message
 */
void send_initial_message(void) {
    uint8_t len = strlen(initial_message);
    UART_Transmit(&UART_0, (uint8_t *)initial_message, len);
}

/**
 * Periodic interrupt handler (1ms period)
 * Handles system timing, ADC sampling, and LED control
 */
void interrupcao_periodica(void) {
    static uint16_t adc_timer = 0;
    static uint32_t time_counter = 0;

    /* Increment counters */
    time_counter++;
    interrupt_count++;

    /* ADC sampling every 2ms */
    adc_timer++;
    if (adc_timer >= 2) {
        adc_flag = 1;
        adc_timer = 0;
    }

    /* Update system time every second */
    if (time_counter >= 1000) {
        system_time.seconds++;
        if (system_time.seconds >= 60) {
            system_time.seconds = 0;
            system_time.minute++;
        }
        if (system_time.minute >= 60) {
            system_time.minute = 0;
            system_time.hour++;
        }
        if (system_time.hour >= 24) {
            system_time.hour = 0;
            system_time.day++;
        }
        time_counter = 0;

        /* Re-enable GPS if no data is ready */
        if (!gps_data_ready && !gps_processing_enabled) {
            gps_processing_enabled = 1;
            memset(&gps_data, 0, sizeof(GPS_Data_t));
            strcpy(gps_data.location, "Unknown");
            memset(&gps_data_int, 0, sizeof(GPS_Data_Int_t));
            strcpy(gps_data_int.location, "Unknown");
            gps_index = 0;
        }
    }

    /* Update ADC measurement */
    if (adc_flag == 1) {
        result = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Channel_A);
        AdcMeasurementHandler();
        adc_flag = 0;
    }

    /* LED blinking control */
    if (led_mode == 1) {
        if (interrupt_count >= tempo) {
            DIGITAL_IO_ToggleOutput(&LED1);
            interrupt_count = 0;
        }
    }
}

/**
 * Button interrupt handler - Toggle LED mode
 */
void Button(void) {
    led_mode = !led_mode;
    interrupt_count = 0;

    /* Update LED state immediately */
    if (led_mode == 0) {
        DIGITAL_IO_SetOutputHigh(&LED1);  // ON mode
    } else {
        DIGITAL_IO_SetOutputLow(&LED1);   // Prepare for blinking
    }
}

/**
 * ADC measurement handler - Map ADC value to LED blinking period
 */
void AdcMeasurementHandler(void) {
    /* Map ADC result (0-4095) to delay (50ms-3000ms) */
    tempo = (result * (3000 - 50) / 4095) + 50;
}

/******************************************************************************
 * UART FUNCTIONS IMPLEMENTATION
 ******************************************************************************/

/**
 * Check for incoming UART data and buffer it
 */
void check_uart_data(void) {
    uint8_t data;
    UART_STATUS_t status;

    while ((status = UART_Receive(&UART_0, &data, 1)) == UART_STATUS_SUCCESS) {
        /* Buffer printable characters */
        if (data != '\r' && data != '\n') {
            if (uart_rx_index < UART_BUFFER_SIZE - 1) {
                uart_rx_buffer[uart_rx_index++] = data;
            } else {
                /* Buffer overflow - reset */
                uart_rx_index = 0;
                memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
                UART_Transmit(&UART_0, (uint8_t *)"\r\nBuffer overflow! Command too long.\r\n",
                            strlen("\r\nBuffer overflow! Command too long.\r\n"));
            }
        }
        /* Process command on CR/LF */
        else if (uart_rx_index > 0) {
            uart_rx_buffer[uart_rx_index] = '\0';
            uart_command_ready = 1;
            UART_Transmit(&UART_0, (uint8_t *)"\r\n", 2);
            break;
        }
    }
}

/**
 * Process received UART commands
 */
void process_uart_command(void) {
    float temperature, humidity;

    if (strcmp((char*)uart_rx_buffer, "GET_ADC") == 0) {
        /* Get current ADC value */
        sprintf(uart_tx_buffer, "ADC Value: %d\r\n", result);
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
    }
    else if (strcmp((char*)uart_rx_buffer, "GET_BLINK") == 0) {
        /* Get LED blinking period */
        sprintf(uart_tx_buffer, "Blinking cadence: %lu ms\r\n", tempo);
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
    }
    else if (strcmp((char*)uart_rx_buffer, "GET_STATUS") == 0) {
        /* Get LED mode status */
        sprintf(uart_tx_buffer, "LED mode: %s\r\n", led_mode ? "Blinking" : "ON");
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
    }
    else if (strcmp((char*)uart_rx_buffer, "GET_GPS") == 0) {
        if (gps_data_ready && gps_stored_data_int.valid_data) {
            /* Display stored GPS data */
            char formatted_time[9] = "00:00:00";
            if (strlen(gps_stored_data_int.time_utc) >= 6) {
                /* Convert UTC to local time (UTC+1 for Portugal summer time) */
                int hours = (gps_stored_data_int.time_utc[0] - '0') * 10 +
                           (gps_stored_data_int.time_utc[1] - '0');
                int minutes = (gps_stored_data_int.time_utc[2] - '0') * 10 +
                             (gps_stored_data_int.time_utc[3] - '0');
                int seconds = (gps_stored_data_int.time_utc[4] - '0') * 10 +
                             (gps_stored_data_int.time_utc[5] - '0');

                hours = (hours + 1) % 24;  // UTC+1 adjustment

                sprintf(formatted_time, "%02d:%02d:%02d", hours, minutes, seconds);
            }

            /* Format date as DD/MM/YYYY */
            char formatted_date[11] = "00/00/0000";
            if (strlen(gps_stored_data_int.date) >= 6) {
                formatted_date[0] = gps_stored_data_int.date[0];
                formatted_date[1] = gps_stored_data_int.date[1];
                formatted_date[2] = '/';
                formatted_date[3] = gps_stored_data_int.date[2];
                formatted_date[4] = gps_stored_data_int.date[3];
                formatted_date[5] = '/';
                formatted_date[6] = '2';
                formatted_date[7] = '0';
                formatted_date[8] = gps_stored_data_int.date[4];
                formatted_date[9] = gps_stored_data_int.date[5];
                formatted_date[10] = '\0';
            }

            /* Format coordinates for display */
            int32_t lat_deg = gps_stored_data_int.latitude_int / 1000000;
            int32_t lat_dec = (gps_stored_data_int.latitude_int < 0 ? -gps_stored_data_int.latitude_int : gps_stored_data_int.latitude_int) % 1000000;
            int32_t lon_deg = gps_stored_data_int.longitude_int / 1000000;
            int32_t lon_dec = (gps_stored_data_int.longitude_int < 0 ? -gps_stored_data_int.longitude_int : gps_stored_data_int.longitude_int) % 1000000;
            int32_t alt_m = gps_stored_data_int.altitude_int / 10;
            int32_t alt_dec = gps_stored_data_int.altitude_int % 10;

            sprintf(uart_tx_buffer,
                    "GPS Fixed!\r\n"
                    "Location: %s\r\n"
                    "Hours: %s\r\n"
                    "Date: %s\r\n"
                    "Altitude: %ld.%ld m above sea level\r\n"
                    "Latitude: %ld.%06ld\r\n"
                    "Longitude: %ld.%06ld\r\n"
                    "Satellites: %d\r\n",
                    gps_stored_data_int.location,
                    formatted_time,
                    formatted_date,
                    alt_m, alt_dec,
                    lat_deg, lat_dec,
                    lon_deg, lon_dec,
                    gps_stored_data_int.satellites);
            UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
        } else {
            /* Start GPS data capture */
            UART_Transmit(&UART_0, (uint8_t *)"Starting GPS data capture...\r\n",
                        strlen("Starting GPS data capture...\r\n"));

            /* Clear current GPS data */
            memset(&gps_data, 0, sizeof(GPS_Data_t));
            memset(&gps_data_int, 0, sizeof(GPS_Data_Int_t));
            strcpy(gps_data.location, "Unknown");
            strcpy(gps_data_int.location, "Unknown");
            gps_index = 0;

            /* Enable GPS processing */
            gps_processing_enabled = 1;
            gps_data_ready = 0;

            UART_Transmit(&UART_0, (uint8_t *)"Waiting for fix...\r\n",
                        strlen("Waiting for fix...\r\n"));
        }
    }
    else if (strcmp((char*)uart_rx_buffer, CMD_GET_TEMPERATURE) == 0 ||
             strcmp((char*)uart_rx_buffer, "GET_TEMP") == 0) {
        /* Read temperature from AHT10 */
        UART_Transmit(&UART_0, (uint8_t *)"Reading temperature...\r\n",
                    strlen("Reading temperature...\r\n"));

        if (!sensor_initialized) {
            UART_Transmit(&UART_0, (uint8_t *)"Sensor not initialized. Trying to initialize...\r\n",
                        strlen("Sensor not initialized. Trying to initialize...\r\n"));
            sensor_initialized = AHT10_Init();
            if (!sensor_initialized) {
                UART_Transmit(&UART_0, (uint8_t *)"Error: Failed to initialize sensor\r\n",
                            strlen("Error: Failed to initialize sensor\r\n"));
                memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
                uart_rx_index = 0;
                uart_command_ready = 0;
                return;
            }
        }

        if (AHT10_ReadValues(&temperature, &humidity)) {
            int temp_int = (int)temperature;
            int temp_frac = (int)((temperature - temp_int) * 100);
            if (temp_frac < 0) temp_frac = -temp_frac;

            sprintf(uart_tx_buffer, "Temperature: %d.%02d °C\r\n", temp_int, temp_frac);
            UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
        } else {
            UART_Transmit(&UART_0, (uint8_t *)"Failed to read temperature! Reinitializing sensor...\r\n",
                        strlen("Failed to read temperature! Reinitializing sensor...\r\n"));
            sensor_initialized = AHT10_Init();
            if (sensor_initialized) {
                UART_Transmit(&UART_0, (uint8_t *)"Sensor reinitialized. Try again.\r\n",
                            strlen("Sensor reinitialized. Try again.\r\n"));
            }
        }
    }
    else if (strcmp((char*)uart_rx_buffer, CMD_GET_HUMIDITY) == 0 ||
             strcmp((char*)uart_rx_buffer, "GET_HUM") == 0) {
        /* Read humidity from AHT10 */
        UART_Transmit(&UART_0, (uint8_t *)"Reading humidity...\r\n",
                    strlen("Reading humidity...\r\n"));

        if (!sensor_initialized) {
            UART_Transmit(&UART_0, (uint8_t *)"Sensor not initialized. Trying to initialize...\r\n",
                        strlen("Sensor not initialized. Trying to initialize...\r\n"));
            sensor_initialized = AHT10_Init();
            if (!sensor_initialized) {
                UART_Transmit(&UART_0, (uint8_t *)"Error: Failed to initialize sensor\r\n",
                            strlen("Error: Failed to initialize sensor\r\n"));
                memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
                uart_rx_index = 0;
                uart_command_ready = 0;
                return;
            }
        }

        if (AHT10_ReadValues(&temperature, &humidity)) {
            int hum_int = (int)humidity;
            int hum_frac = (int)((humidity - hum_int) * 100);

            sprintf(uart_tx_buffer, "Humidity: %d.%02d %%\r\n", hum_int, hum_frac);
            UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
        } else {
            UART_Transmit(&UART_0, (uint8_t *)"Failed to read humidity! Reinitializing sensor...\r\n",
                        strlen("Failed to read humidity! Reinitializing sensor...\r\n"));
            sensor_initialized = AHT10_Init();
            if (sensor_initialized) {
                UART_Transmit(&UART_0, (uint8_t *)"Sensor reinitialized. Try again.\r\n",
                            strlen("Sensor reinitialized. Try again.\r\n"));
            }
        }
    }
    else if (strcmp((char*)uart_rx_buffer, "GPS_DEBUG") == 0) {
        /* Show GPS debug information */
        sprintf(uart_tx_buffer,
                "GPS Debug Info:\r\n"
                "Processing enabled: %s\r\n"
                "Buffer index: %d\r\n"
                "Valid data: %d\r\n"
                "Fix quality: %d\r\n"
                "Data ready: %d\r\n"
                "Stored valid: %d\r\n",
                gps_processing_enabled ? "YES" : "NO",
                gps_index,
                gps_data_int.valid_data,
                gps_data_int.fix_quality,
                gps_data_ready,
                gps_stored_data_int.valid_data);
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
    }
    else if (strcmp((char*)uart_rx_buffer, "GPS_RESET") == 0) {
        /* Reset GPS processing */
        gps_processing_enabled = 0;
        gps_data_ready = 0;
        gps_index = 0;
        memset(&gps_data, 0, sizeof(GPS_Data_t));
        memset(&gps_stored_data, 0, sizeof(GPS_Data_t));
        memset(&gps_data_int, 0, sizeof(GPS_Data_Int_t));
        memset(&gps_stored_data_int, 0, sizeof(GPS_Data_Int_t));
        strcpy(gps_data.location, "Unknown");
        strcpy(gps_stored_data.location, "Unknown");
        strcpy(gps_data_int.location, "Unknown");
        strcpy(gps_stored_data_int.location, "Unknown");
        UART_Transmit(&UART_0, (uint8_t *)"GPS processing reset!\r\n",
                    strlen("GPS processing reset!\r\n"));
    }
    else if (strcmp((char*)uart_rx_buffer, "GPS_TEST") == 0) {
        /* Test GPS conversion functions */
        int32_t lat_int, lon_int, alt_int;

        /* Test with sample coordinates */
        nmea_to_int("4110.88520", 0, &lat_int);
        nmea_to_int("00808.81874", 1, &lon_int);
        parse_altitude_int("221.3", &alt_int);

        sprintf(uart_tx_buffer,
                "GPS Conversion Test:\r\n"
                "Input: 4110.88520,N / 00808.81874,W\r\n"
                "Latitude:  %ld.%06ld° (expected: 41.181420°)\r\n"
                "Longitude: %ld.%06ld° (expected: -8.147312°)\r\n"
                "Altitude: %ld.%ld m\r\n",
                lat_int / 1000000, lat_int % 1000000,
                lon_int / 1000000, (lon_int < 0 ? -lon_int : lon_int) % 1000000,
                alt_int / 10, alt_int % 10);
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));

        /* Test GPRMC parsing */
        strcpy(gps_buffer, "$GPRMC,021153.00,A,4110.88520,N,00808.81874,W,0.071,,190625,,,A*6E");
        GPS_ProcessData();

        sprintf(uart_tx_buffer,
                "GPRMC Parse Test Result:\r\n"
                "Parsed Lat: %ld.%06ld°\r\n"
                "Parsed Lon: %ld.%06ld°\r\n"
                "Location: %s\r\n",
                gps_data_int.latitude_int / 1000000,
                gps_data_int.latitude_int % 1000000,
                gps_data_int.longitude_int / 1000000,
                (gps_data_int.longitude_int < 0 ? -gps_data_int.longitude_int : gps_data_int.longitude_int) % 1000000,
                gps_data_int.location);
        UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
    }
    else if (strcmp((char*)uart_rx_buffer, "SENSOR_TEST") == 0) {
        /* Test sensor with multiple readings */
        UART_Transmit(&UART_0, (uint8_t *)"Testing AHT10 sensor (3 readings)...\r\n",
                      strlen("Testing AHT10 sensor (3 readings)...\r\n"));

        for (int i = 0; i < 3; i++) {
            float temp, hum;

            sprintf(uart_tx_buffer, "\r\nReading %d:\r\n", i+1);
            UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));

            /* Small delay before reading */
            for(volatile uint32_t j = 0; j < 100000; j++);

            if (AHT10_ReadValues(&temp, &hum)) {
                /* Convert to integers for display */
                int temp_int = (int)temp;
                int temp_frac = (int)((temp - temp_int) * 100);
                if (temp_frac < 0) temp_frac = -temp_frac;

                int hum_int = (int)hum;
                int hum_frac = (int)((hum - hum_int) * 100);

                sprintf(uart_tx_buffer, "Temp: %d.%02d°C, Hum: %d.%02d%%\r\n",
                        temp_int, temp_frac, hum_int, hum_frac);
                UART_Transmit(&UART_0, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
            } else {
                UART_Transmit(&UART_0, (uint8_t *)"Failed to read sensor!\r\n",
                              strlen("Failed to read sensor!\r\n"));
            }

            /* Delay between readings */
            for(volatile uint32_t j = 0; j < 500000; j++);
        }
    }
    else if (strcmp((char*)uart_rx_buffer, "GPS_STOP") == 0) {
        /* Stop GPS processing */
        gps_processing_enabled = 0;
        UART_Transmit(&UART_0, (uint8_t *)"GPS processing stopped.\r\n",
                      strlen("GPS processing stopped.\r\n"));
    }
    else if (strcmp((char*)uart_rx_buffer, "HELP") == 0) {
        /* Show help menu */
        const char* help_msg = "Available commands:\r\n"
                              "GET_ADC - Query ADC value\r\n"
                              "GET_BLINK - Query blinking cadence\r\n"
                              "GET_STATUS - Query LED status\r\n"
                              "GET_TEMP - Read temperature in Celsius\r\n"
                              "GET_HUM - Read humidity percentage\r\n"
                              "GET_GPS - Read GPS position\r\n"
                              "GPS_DEBUG - Show GPS debug info\r\n"
                              "GPS_RESET - Reset GPS processing\r\n"
                              "GPS_STOP - Stop GPS processing\r\n"
                              "GPS_TEST - Test GPS parsing functions\r\n"
                              "HELP - Show this help\r\n";
        UART_Transmit(&UART_0, (uint8_t *)help_msg, strlen(help_msg));
    }
    else {
        /* Unknown command */
        const char* unknown_msg = "Unknown command. Type HELP to see available commands.\r\n";
        UART_Transmit(&UART_0, (uint8_t *)unknown_msg, strlen(unknown_msg));
    }

    /* Clear buffer and reset flags */
    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
    uart_rx_index = 0;
    uart_command_ready = 0;
}

/******************************************************************************
 * AHT10 SENSOR FUNCTIONS IMPLEMENTATION
 ******************************************************************************/

/**
 * Reset AHT10 sensor
 */
void AHT10_Reset(void) {
    i2c_tx_buffer[0] = AHT10_CMD_RESET;
    I2C_MASTER_Transmit(&I2C_MASTER_0, true, AHT10_I2C_ADDR, i2c_tx_buffer, 1, true);
    /* Increased delay for proper reset (20ms minimum per datasheet) */
    for(volatile uint32_t i = 0; i < 200000; i++);
}

/**
 * Initialize AHT10 sensor
 * Returns: 1 if successful, 0 if failed
 */
uint8_t AHT10_Init(void) {
    /* Reset sensor */
    AHT10_Reset();

    /* Delay after reset (40ms to be safe) */
    for(volatile uint32_t i = 0; i < 400000; i++);

    /* Send initialization command */
    i2c_tx_buffer[0] = AHT10_CMD_INIT;
    i2c_tx_buffer[1] = 0x08;
    i2c_tx_buffer[2] = 0x00;

    I2C_MASTER_STATUS_t status = I2C_MASTER_Transmit(&I2C_MASTER_0, true,
                                                     AHT10_I2C_ADDR, i2c_tx_buffer, 3, true);
    if (status != I2C_MASTER_STATUS_SUCCESS) {
        return 0;
    }

    /* Delay after initialization (10ms minimum) */
    for(volatile uint32_t i = 0; i < 100000; i++);

    return 1;
}

/**
 * Read temperature and humidity from AHT10
 * Returns: 1 if successful, 0 if failed
 */
uint8_t AHT10_ReadValues(float *temperature, float *humidity) {
    I2C_MASTER_STATUS_t status;

    /* Send measurement command */
    i2c_tx_buffer[0] = AHT10_CMD_MEASURE;
    i2c_tx_buffer[1] = 0x33;
    i2c_tx_buffer[2] = 0x00;

    status = I2C_MASTER_Transmit(&I2C_MASTER_0, true, AHT10_I2C_ADDR, i2c_tx_buffer, 3, true);
    if (status != I2C_MASTER_STATUS_SUCCESS) {
        return 0;
    }

    /* Wait for measurement (80ms minimum per datasheet) */
    for(volatile uint32_t i = 0; i < 800000; i++);

    /* Read measurement data */
    status = I2C_MASTER_Receive(&I2C_MASTER_0, true, AHT10_I2C_ADDR, i2c_rx_buffer, 6, true, true);
    if (status != I2C_MASTER_STATUS_SUCCESS) {
        return 0;
    }

    /* Check if sensor is still busy */
    if (i2c_rx_buffer[0] & AHT10_STATUS_BUSY) {
        /* Wait a bit more and try reading again */
        for(volatile uint32_t i = 0; i < 200000; i++);

        status = I2C_MASTER_Receive(&I2C_MASTER_0, true, AHT10_I2C_ADDR, i2c_rx_buffer, 6, true, true);
        if (status != I2C_MASTER_STATUS_SUCCESS) {
            return 0;
        }

        /* If still busy, fail */
        if (i2c_rx_buffer[0] & AHT10_STATUS_BUSY) {
            return 0;
        }
    }

    /* Extract humidity (20 bits) */
    uint32_t humidity_raw = ((uint32_t)i2c_rx_buffer[1] << 12) |
                           ((uint32_t)i2c_rx_buffer[2] << 4) |
                           (i2c_rx_buffer[3] >> 4);

    /* Extract temperature (20 bits) */
    uint32_t temperature_raw = ((uint32_t)(i2c_rx_buffer[3] & 0x0F) << 16) |
                               ((uint32_t)i2c_rx_buffer[4] << 8) |
                              i2c_rx_buffer[5];

    /* Convert to actual values using the correct formula */
    *humidity = (float)humidity_raw * 100.0f / 1048576.0f;
    *temperature = (float)temperature_raw * 200.0f / 1048576.0f - 50.0f;

    /* Validate readings */
    if (*humidity > 100.0f || *humidity < 0.0f ||
        *temperature > 85.0f || *temperature < -40.0f) {
        return 0;  /* Invalid readings */
    }

    /* Store last readings */
    last_temperature = *temperature;
    last_humidity = *humidity;

    return 1;
}
    /******************************************************************************
     * GPS FUNCTIONS IMPLEMENTATION
     ******************************************************************************/

    /**
     * Initialize GPS module
     */
    void GPS_Init(void) {
        memset(&gps_data, 0, sizeof(GPS_Data_t));
        memset(&gps_stored_data, 0, sizeof(GPS_Data_t));
        memset(&gps_data_int, 0, sizeof(GPS_Data_Int_t));
        memset(&gps_stored_data_int, 0, sizeof(GPS_Data_Int_t));
        gps_index = 0;
        gps_processing_enabled = 0;
        gps_data_ready = 0;
        strcpy(gps_data.location, "Unknown");
        strcpy(gps_stored_data.location, "Unknown");
        strcpy(gps_data_int.location, "Unknown");
        strcpy(gps_stored_data_int.location, "Unknown");
        UART_Transmit(&UART_0, (uint8_t *)"GPS OK\r\n", 8);
    }

    /**
     * Convert NMEA coordinate to integer (multiplied by 1000000)
     * @param nmea_str: NMEA coordinate string
     * @param is_longitude: 1 for longitude (DDD), 0 for latitude (DD)
     * @param result: Output integer result
     */
    void nmea_to_int(const char* nmea_str, int is_longitude, int32_t* result) {
        int32_t degrees = 0;
        int32_t minutes_whole = 0;
        int32_t minutes_frac = 0;
        int idx = 0;

        *result = 0;

        if (nmea_str == NULL || strlen(nmea_str) < 5) return;

        /* Parse degrees */
        if (is_longitude) {
            /* Longitude: DDD (3 digits) */
            if (nmea_str[0] >= '0' && nmea_str[0] <= '9' &&
                nmea_str[1] >= '0' && nmea_str[1] <= '9' &&
                nmea_str[2] >= '0' && nmea_str[2] <= '9') {
                degrees = (nmea_str[0] - '0') * 100 + (nmea_str[1] - '0') * 10 + (nmea_str[2] - '0');
                idx = 3;
            }
        } else {
            /* Latitude: DD (2 digits) */
            if (nmea_str[0] >= '0' && nmea_str[0] <= '9' &&
                nmea_str[1] >= '0' && nmea_str[1] <= '9') {
                degrees = (nmea_str[0] - '0') * 10 + (nmea_str[1] - '0');
                idx = 2;
            }
        }

        /* Parse minutes integer part */
        if (nmea_str[idx] >= '0' && nmea_str[idx] <= '9' &&
            nmea_str[idx+1] >= '0' && nmea_str[idx+1] <= '9') {
            minutes_whole = (nmea_str[idx] - '0') * 10 + (nmea_str[idx+1] - '0');
            idx += 2;
        }

        /* Parse decimal point and fractional minutes */
        minutes_frac = 0;
        if (nmea_str[idx] == '.') {
            idx++;
            int digits = 0;
            int multiplier = 1000;

            while (digits < 4 && nmea_str[idx] >= '0' && nmea_str[idx] <= '9') {
                minutes_frac += (nmea_str[idx] - '0') * multiplier;
                multiplier /= 10;
                idx++;
                digits++;
            }
        }

        /* Convert minutes to decimal degrees with proper precision */
        int32_t total_minutes_scaled = minutes_whole * 10000 + minutes_frac;

        /* Use 64-bit arithmetic to avoid overflow */
        int64_t temp = (int64_t)total_minutes_scaled * 1000000;
        int32_t decimal_part = (int32_t)(temp / 600000);

        /* Final result in microdegrees */
        *result = degrees * 1000000 + decimal_part;
    }

    /**
     * Simple string to integer conversion
     */
    int simple_atoi(const char* str) {
        int result = 0;
        int i = 0;

        if (str == NULL) return 0;

        while (str[i] >= '0' && str[i] <= '9') {
            result = result * 10 + (str[i] - '0');
            i++;
        }

        return result;
    }

    /**
     * Parse altitude string to integer (altitude * 10)
     */
    void parse_altitude_int(const char* str, int32_t* altitude) {
        int32_t int_part = 0;
        int32_t frac_part = 0;
        int idx = 0;

        *altitude = 0;

        if (str == NULL || strlen(str) == 0) return;

        /* Parse integer part */
        while (str[idx] >= '0' && str[idx] <= '9') {
            int_part = int_part * 10 + (str[idx] - '0');
            idx++;
        }

        /* Parse one decimal place */
        if (str[idx] == '.' && str[idx+1] >= '0' && str[idx+1] <= '9') {
            frac_part = str[idx+1] - '0';
        }

        *altitude = int_part * 10 + frac_part;
    }

    /**
     * Get location name from GPS coordinates
     */
    void GPS_GetLocationFromCoordinates_Int(int32_t lat_int, int32_t lon_int, char* location) {
        /* Porto area: 41.10° to 41.20°N, 8.50° to 8.70°W */
        if (lat_int >= 41100000 && lat_int <= 41200000 &&
            lon_int >= -8700000 && lon_int <= -8500000) {
            strcpy(location, "Porto, Portugal");
        }
        /* Lisboa area: 38.70° to 38.78°N, 9.10° to 9.20°W */
        else if (lat_int >= 38700000 && lat_int <= 38780000 &&
                 lon_int >= -9200000 && lon_int <= -9100000) {
            strcpy(location, "Lisboa, Portugal");
        }
        /* Braga area: 41.53° to 41.56°N, 8.40° to 8.45°W */
        else if (lat_int >= 41530000 && lat_int <= 41560000 &&
                 lon_int >= -8450000 && lon_int <= -8400000) {
            strcpy(location, "Braga, Portugal");
        }
        /* Coimbra area: 40.19° to 40.23°N, 8.40° to 8.45°W */
        else if (lat_int >= 40190000 && lat_int <= 40230000 &&
                 lon_int >= -8450000 && lon_int <= -8400000) {
            strcpy(location, "Coimbra, Portugal");
        }
        /* Test area based on coordinates: ~41.181°, ~-8.147° */
        else if (lat_int >= 41170000 && lat_int <= 41190000 &&
                 lon_int >= -8160000 && lon_int <= -8130000) {
            strcpy(location, "Porto Region, Portugal");
        }
        else {
            /* Format coordinates as readable string */
            int lat_deg = lat_int / 1000000;
            int lat_dec = (lat_int < 0 ? -lat_int : lat_int) % 1000000;
            int lon_deg = lon_int / 1000000;
            int lon_dec = (lon_int < 0 ? -lon_int : lon_int) % 1000000;

            sprintf(location, "Lat: %d.%06d, Lon: %d.%06d",
                    lat_deg, lat_dec, lon_deg, lon_dec);
        }
    }

    /**
     * Read GPS data from UART
     */
    void GPS_ReadData(void) {
        uint8_t byte;
        static uint8_t consecutive_sentences = 0;

        /* Only process if GPS is enabled */
        if (!gps_processing_enabled) {
            return;
        }

        /* Read available bytes with limit to avoid blocking */
        int bytes_read = 0;
        const int MAX_BYTES_PER_CALL = 500;

        while (bytes_read < MAX_BYTES_PER_CALL &&
               UART_Receive(&UART_1, &byte, 1) == UART_STATUS_SUCCESS) {
            bytes_read++;

            /* Start new sentence on '$' */
            if (byte == '$') {
                gps_index = 0;
                gps_buffer[gps_index++] = byte;
            } else if (gps_index > 0) {
                if (gps_index < GPS_BUFFER_SIZE - 1) {
                    gps_buffer[gps_index++] = byte;
                }

                /* Process complete sentence on newline */
                if (byte == '\n') {
                    gps_buffer[gps_index] = '\0';
                    GPS_ProcessData();
                    gps_index = 0;
                    consecutive_sentences++;

                    /* Check if we have complete data */
                    if (gps_data_int.valid_data && gps_data_int.fix_quality > 0) {
                        if ((gps_data_int.altitude_int > 0 && gps_data_int.satellites > 0) ||
                            consecutive_sentences > 50) {

                            /* Store captured data */
                            memcpy(&gps_stored_data, &gps_data, sizeof(GPS_Data_t));
                            memcpy(&gps_stored_data_int, &gps_data_int, sizeof(GPS_Data_Int_t));
                            gps_data_ready = 1;
                            gps_processing_enabled = 0;
                            consecutive_sentences = 0;

                            /* Notify successful capture */
                            char msg[150];
                            sprintf(msg, "GPS: Data captured - Lat:%ld.%06ld, Lon:%ld.%06ld, Alt:%ld.%ldm, Sats:%d\r\n",
                                    (long)gps_stored_data_int.latitude_int / 1000000,
                                    (long)(gps_stored_data_int.latitude_int < 0 ? -gps_stored_data_int.latitude_int : gps_stored_data_int.latitude_int) % 1000000,
                                    (long)gps_stored_data_int.longitude_int / 1000000,
                                    (long)(gps_stored_data_int.longitude_int < 0 ? -gps_stored_data_int.longitude_int : gps_stored_data_int.longitude_int) % 1000000,
                                    (long)gps_stored_data_int.altitude_int / 10,
                                    (long)gps_stored_data_int.altitude_int % 10,
                                    gps_stored_data_int.satellites);
                            UART_Transmit(&UART_0, (uint8_t *)msg, strlen(msg));
                            return;
                        }
                    }
                }
            }
        }
    }

    /**
     * Process GPS NMEA sentences
     */
    void GPS_ProcessData(void) {
        if (strlen(gps_buffer) < 20 || gps_buffer[0] != '$') {
            return;
        }

        /* Process GPRMC sentence */
        if (strncmp(gps_buffer, "$GPRMC", 6) == 0 || strncmp(gps_buffer, "$GNRMC", 6) == 0) {
            char temp[512];
            strcpy(temp, gps_buffer);
            char* parts[20];
            int field_count = 0;

            /* Parse comma-separated fields */
            int start = 0;
            int pos = 0;
            while (temp[pos] != '\0' && field_count < 20) {
                if (temp[pos] == ',') {
                    temp[pos] = '\0';
                    parts[field_count++] = &temp[start];
                    start = pos + 1;
                }
                pos++;
            }
            if (start < pos) {
                parts[field_count++] = &temp[start];
            }

            if (field_count >= 10) {
                /* UTC Time */
                if (strlen(parts[1]) >= 6) {
                    strncpy(gps_data_int.time_utc, parts[1], 10);
                    gps_data_int.time_utc[10] = '\0';
                    strncpy(gps_data.time_utc, parts[1], 10);
                    gps_data.time_utc[10] = '\0';
                }

                /* Status - A=Active/Valid, V=Void/Invalid */
                if (parts[2][0] == 'A') {
                    gps_data_int.valid_data = 1;
                    gps_data.valid_data = 1;

                    /* Latitude */
                    if (strlen(parts[3]) > 0 && strlen(parts[4]) > 0) {
                        nmea_to_int(parts[3], 0, &gps_data_int.latitude_int);

                        if (parts[4][0] == 'S') {
                            gps_data_int.latitude_int = -gps_data_int.latitude_int;
                        }

                        gps_data.latitude = gps_data_int.latitude_int / 1000000.0;
                    }

                    /* Longitude */
                    if (strlen(parts[5]) > 0 && strlen(parts[6]) > 0) {
                        nmea_to_int(parts[5], 1, &gps_data_int.longitude_int);

                        if (parts[6][0] == 'W') {
                            gps_data_int.longitude_int = -gps_data_int.longitude_int;
                        }

                        gps_data.longitude = gps_data_int.longitude_int / 1000000.0;
                    }

                    /* Speed in knots */
                    if (strlen(parts[7]) > 0) {
                        float speed = 0;
                        int whole = 0;
                        int frac = 0;
                        int decimal_pos = -1;

                        for (int i = 0; parts[7][i] != '\0'; i++) {
                            if (parts[7][i] == '.') {
                                decimal_pos = i;
                            } else if (parts[7][i] >= '0' && parts[7][i] <= '9') {
                                if (decimal_pos < 0) {
                                    whole = whole * 10 + (parts[7][i] - '0');
                                } else if (i - decimal_pos <= 3) {
                                    frac = frac * 10 + (parts[7][i] - '0');
                                }
                            }
                        }
                        speed = whole + (float)frac / 1000.0;
                        gps_data_int.speed_knots_int = (int32_t)(speed * 1000);
                        gps_data.speed_knots = speed;
                    }

                    /* Date */
                    if (field_count > 9 && strlen(parts[9]) >= 6) {
                        strncpy(gps_data_int.date, parts[9], 6);
                        gps_data_int.date[6] = '\0';
                        strncpy(gps_data.date, parts[9], 6);
                        gps_data.date[6] = '\0';
                    }

                    gps_data_int.fix_quality = 1;
                    gps_data.fix_quality = 1;

                    /* Get location name */
                    GPS_GetLocationFromCoordinates_Int(gps_data_int.latitude_int,
                                                      gps_data_int.longitude_int,
                                                      gps_data_int.location);
                    gps_data_int.location_found = 1;
                    strcpy(gps_data.location, gps_data_int.location);
                    gps_data.location_found = 1;
                }
            }
        }
        /* Process GPGGA sentence for altitude and satellites */
        else if (strncmp(gps_buffer, "$GPGGA", 6) == 0 || strncmp(gps_buffer, "$GNGGA", 6) == 0) {
            char temp[512];
            strcpy(temp, gps_buffer);
            char* parts[20];
            int field_count = 0;

            /* Parse comma-separated fields */
            int start = 0;
            int pos = 0;
            while (temp[pos] != '\0' && field_count < 20) {
                if (temp[pos] == ',') {
                    temp[pos] = '\0';
                    parts[field_count++] = &temp[start];
                    start = pos + 1;
                }
                pos++;
            }
            if (start < pos) {
                parts[field_count++] = &temp[start];
            }

            if (field_count >= 11) {
                /* Fix quality (0=invalid, 1=GPS, 2=DGPS) */
                if (strlen(parts[6]) > 0) {
                    gps_data_int.fix_quality = parts[6][0] - '0';
                    gps_data.fix_quality = gps_data_int.fix_quality;
                }

                /* Number of satellites */
                if (strlen(parts[7]) > 0) {
                    gps_data_int.satellites = simple_atoi(parts[7]);
                    gps_data.satellites = gps_data_int.satellites;
                }

                /* Altitude above sea level */
                if (gps_data_int.fix_quality > 0 && strlen(parts[9]) > 0) {
                    parse_altitude_int(parts[9], &gps_data_int.altitude_int);
                    gps_data.altitude = gps_data_int.altitude_int / 10.0;
                }
            }
        }
    }

    /**
     * Get current GPS data pointer
     */
    GPS_Data_t* GPS_GetData(void) {
        return &gps_data;
    }

    /******************************************************************************
     * MAIN FUNCTION
     ******************************************************************************/

    int main(void) {
        DAVE_STATUS_t status;

        /* Initialize DAVE Apps */
        status = DAVE_Init();
        if (status == DAVE_STATUS_FAILURE) {
            XMC_DEBUG(("DAVE Apps initialization failed with status %d\n", status));
            while (1U);
        }

        /* Send initial identification message */
        send_initial_message();

        /* Send welcome message */
        const char* welcome_msg = "System started. Type HELP to see available commands.\r\n";
        UART_Transmit(&UART_0, (uint8_t *)welcome_msg, strlen(welcome_msg));

        /* Initialize AHT10 temperature/humidity sensor */
        UART_Transmit(&UART_0, (uint8_t *)"Initializing AHT10 sensor...\r\n",
                      strlen("Initializing AHT10 sensor...\r\n"));

        /* Longer delay for sensor power-up stabilization */
        for(volatile uint32_t i = 0; i < 1000000; i++);

        sensor_initialized = AHT10_Init();
        if (sensor_initialized) {
            UART_Transmit(&UART_0, (uint8_t *)"AHT10 sensor initialized successfully!\r\n",
                          strlen("AHT10 sensor initialized successfully!\r\n"));
        } else {
            UART_Transmit(&UART_0, (uint8_t *)"Failed to initialize AHT10 sensor. Trying again...\r\n",
                          strlen("Failed to initialize AHT10 sensor. Trying again...\r\n"));

            /* Retry with additional delay */
            for(volatile uint32_t i = 0; i < 500000; i++);

            sensor_initialized = AHT10_Init();
            if (sensor_initialized) {
                UART_Transmit(&UART_0, (uint8_t *)"AHT10 sensor initialized successfully on second attempt!\r\n",
                              strlen("AHT10 sensor initialized successfully on second attempt!\r\n"));
            } else {
                UART_Transmit(&UART_0, (uint8_t *)"Sensor initialization failed. Check connections.\r\n",
                              strlen("Sensor initialization failed. Check connections.\r\n"));
            }
        }

        /* Initialize GPS module */
        UART_Transmit(&UART_0, (uint8_t *)"Initializing GPS module...\r\n",
                      strlen("Initializing GPS module...\r\n"));
        GPS_Init();

        /* Enable GPS processing to get initial fix */
        gps_processing_enabled = 1;

        /* Main application loop */
        while (1U) {
            /* Check ADC periodically */
            if (adc_flag == 1) {
                result = ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Channel_A);
                AdcMeasurementHandler();
                adc_flag = 0;
            }

            /* Process GPS data */
            GPS_ReadData();

            /* Check for UART commands */
            check_uart_data();

            /* Process UART commands when ready */
            if (uart_command_ready) {
                process_uart_command();
            }
        }

        return 1;
    }
