#include <ctype.h>
#include "gd32vw55x.h"
#include <stdio.h>
#include "app_es.h"
#include "wrapper_os.h"
#include "dbg_print.h"
#include "wakelock.h"
#include "log_uart.h"
#include "ble_es_srv.h"

#define ES_TASK_PRIORITY OS_TASK_PRIORITY(4)
#define UART_QUEUE_SIZE 3

#define ES_RCU_GPIO_I2C_SCL RCU_GPIOB
#define ES_RCU_GPIO_I2C_SDA RCU_GPIOB
#define ES_RCU_I2C          RCU_I2C0

#define ES_I2CX           I2C0
#define ES_I2C_SCL_GPIO   GPIOA
#define ES_I2C_SCL_PIN    GPIO_PIN_2
#define ES_I2C_SCL_AF_NUM GPIO_AF_4
#define ES_I2C_SDA_GPIO   GPIOA
#define ES_I2C_SDA_PIN    GPIO_PIN_3
#define ES_I2C_SDA_AF_NUM GPIO_AF_4

#define BME280_ADDRESS (0x76 << 1)

#define BME280_REG_ID     0xd0

#define BME280_REG_RESET  0xe0
#define BME280_RESET      0xb6

#define BME280_REG_STATUS     0xf3
#define BME280_STATUS_MEASURING (1 << 3)
#define BME280_STATUS_NVM_BUSY  (1 << 0)

#define BME280_REG_CONTROL_HUMIDITY   0xf2
#define BME280_REG_CONTROL_MEARUEMENT 0xf4

#define BME280_MEASUREMENT_SKIPPED  0
#define BME280_MEASUREMENT_OSR1     1
#define BME280_MEASUREMENT_OSR2     2
#define BME280_MEASUREMENT_OSR4     3
#define BME280_MEASUREMENT_OSR8     4
#define BME280_MEASUREMENT_OSR16    5

#define BME280_MEASUREMENT_OSRT(osr) ((osr) << 5)
#define BME280_MEASUREMENT_OSRP(osr) ((osr) << 2)
#define BME280_MEASUREMENT_OSRH(osr) (osr)

#define BME280_MEASUREMENT_MODE_SLEEP   0x00
#define BME280_MEASUREMENT_MODE_FORCED  0x01
#define BME280_MEASUREMENT_MODE_NORMAL  0x03

#define BME280_REG_CONFIG 0xf5

#define BME280_CONFIG_STANDBY(standby) ((standby) << 5)
#define BME280_CONFIG_STANDBY_0_5   0
#define BME280_CONFIG_STANDBY_62_5  1
#define BME280_CONFIG_STANDBY_125   2
#define BME280_CONFIG_STANDBY_250   3
#define BME280_CONFIG_STANDBY_500   4
#define BME280_CONFIG_STANDBY_1000  5
#define BME280_CONFIG_STANDBY_10    6
#define BME280_CONFIG_STANDBY_20    7

#define BME280_CONFIG_FILTER(filter) ((filter) << 2)
#define BME280_CONFIG_FILTER_OFF  0
#define BME280_CONFIG_FILTER_2    1
#define BME280_CONFIG_FILTER_4    2
#define BME280_CONFIG_FILTER_8    3
#define BME280_CONFIG_FILTER_16   4

#define BME280_REG_CALIBRATION_A  0x88
#define BME280_REG_CALIBRATION_B  0xe1
#define BME280_REG_HUMIDITY     0xfd
#define BME280_REG_TEMPERATURE  0xfa
#define BME280_REG_PRESSURE     0xf7

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calibration_t;

extern uint8_t conn_idx;

static os_queue_t uart_queue;

static bme280_calibration_t calibration;
static int32_t t_fine;

static void bme280_write_multiple(uint8_t *regs, uint8_t *values, size_t count)
{
    I2C_STAT(ES_I2CX) |= I2C_STAT_TBE;
    i2c_master_addressing(ES_I2CX, BME280_ADDRESS, I2C_MASTER_TRANSMIT);
    i2c_transfer_byte_number_config(ES_I2CX, count * 2);
    i2c_automatic_end_enable(ES_I2CX);

    while (i2c_flag_get(ES_I2CX, I2C_FLAG_I2CBSY));
    i2c_start_on_bus(ES_I2CX);

    while (i2c_flag_get(ES_I2CX, I2C_FLAG_TBE) != SET);

    for (int i = 0; i < count; i++) {
        while (!i2c_flag_get(ES_I2CX, I2C_FLAG_TI));
        i2c_data_transmit(ES_I2CX, regs[i]);
        while (!i2c_flag_get(ES_I2CX, I2C_FLAG_TI));
        i2c_data_transmit(ES_I2CX, values[i]);
    }

    while (!i2c_flag_get(ES_I2CX, I2C_FLAG_STPDET));
    i2c_flag_clear(ES_I2CX, I2C_FLAG_STPDET);
}

static void bme280_write(uint8_t reg, uint8_t value)
{
    bme280_write_multiple(&reg, &value, 1);
}

static void bme280_read(uint8_t offset, uint8_t *buffer, size_t count)
{
    I2C_STAT(ES_I2CX) |= I2C_STAT_TBE;
    i2c_master_addressing(ES_I2CX, BME280_ADDRESS, I2C_MASTER_TRANSMIT);
    i2c_transfer_byte_number_config(ES_I2CX, 1);
    i2c_automatic_end_disable(ES_I2CX);

    while (i2c_flag_get(ES_I2CX, I2C_FLAG_I2CBSY));
    i2c_start_on_bus(ES_I2CX);
    while (i2c_flag_get(ES_I2CX, I2C_FLAG_TBE) != SET);

    i2c_data_transmit(ES_I2CX, offset);
    while (!i2c_flag_get(ES_I2CX, I2C_FLAG_TC));

    i2c_master_addressing(ES_I2CX, BME280_ADDRESS, I2C_MASTER_RECEIVE);
    i2c_transfer_byte_number_config(ES_I2CX, count);
    i2c_automatic_end_enable(ES_I2CX);

    i2c_start_on_bus(ES_I2CX);
    for (int i = 0; i < count; i++) {
        while (!i2c_flag_get(ES_I2CX, I2C_FLAG_RBNE));
        buffer[i] = i2c_data_receive(ES_I2CX);
    }

    while (!i2c_flag_get(ES_I2CX, I2C_FLAG_STPDET));
    i2c_flag_clear(ES_I2CX, I2C_FLAG_STPDET);
}

static uint8_t bme280_read_id(void) {
    uint8_t id;
    bme280_read(BME280_REG_ID, &id, 1);
    return id;
}

static uint8_t bme280_read_status(void) {
    uint8_t status;
    bme280_read(BME280_REG_STATUS, &status, 1);
    return status;
}

static uint32_t bme280_read_raw_temperature(void) {
    uint8_t temperature[3];
    bme280_read(BME280_REG_TEMPERATURE, temperature, 3);
    return temperature[0] << 12 | temperature[1] << 4 | temperature[2] >> 4;
}

static uint32_t bme280_read_raw_pressure(void) {
    uint8_t pressure[3];
    bme280_read(BME280_REG_PRESSURE, pressure, 3);
    return pressure[0] << 12 | pressure[1] << 4 | pressure[2] >> 4;
}

static uint16_t bme280_read_raw_humidity(void) {
    uint8_t humidity[2];
    bme280_read(BME280_REG_HUMIDITY, humidity, 2);
    return humidity[0] << 8 | humidity[1];
}

static void bme280_read_calibration(bme280_calibration_t *calibration) {
    uint8_t calib_a[26];

    bme280_read(BME280_REG_CALIBRATION_A, calib_a, sizeof(calib_a));
    
    calibration->dig_T1 = (uint16_t)(calib_a[1] << 8) | calib_a[0];
    calibration->dig_T2 = (int16_t)(calib_a[3] << 8) | calib_a[2];
    calibration->dig_T3 = (int16_t)(calib_a[5] << 8) | calib_a[4];
    calibration->dig_P1 = (uint16_t)(calib_a[7] << 8) | calib_a[6];
    calibration->dig_P2 = (int16_t)(calib_a[9] << 8) | calib_a[8];
    calibration->dig_P3 = (int16_t)(calib_a[11] << 8) | calib_a[10];
    calibration->dig_P4 = (int16_t)(calib_a[13] << 8) | calib_a[12];
    calibration->dig_P5 = (int16_t)(calib_a[15] << 8) | calib_a[14];
    calibration->dig_P6 = (int16_t)(calib_a[17] << 8) | calib_a[16];
    calibration->dig_P7 = (int16_t)(calib_a[19] << 8) | calib_a[18];
    calibration->dig_P8 = (int16_t)(calib_a[21] << 8) | calib_a[20];
    calibration->dig_P9 = (int16_t)(calib_a[23] << 8) | calib_a[22];

    uint8_t calib_b[7];

    bme280_read(BME280_REG_CALIBRATION_B, calib_b, sizeof(calib_b));
    
    calibration->dig_H1 = calib_a[25];
    calibration->dig_H2 = (int16_t)(calib_b[1] << 8) | calib_b[0];
    calibration->dig_H3 = calib_b[2];
    calibration->dig_H4 = (int16_t)(calib_b[3] << 4) | (calib_b[4] & 0x0F);
    calibration->dig_H5 = (int16_t)(calib_b[5] << 4) | (calib_b[4] >> 4);
    calibration->dig_H6 = (int8_t)calib_b[6];
}

static void bme280_configure(uint8_t config) {
    bme280_write(BME280_REG_CONFIG, config);
}

static void bme280_configure_measurement(uint8_t config) {
    bme280_write(BME280_REG_CONTROL_MEARUEMENT, config);
}

static void bme280_configure_humidity(uint8_t config) {
    bme280_write(BME280_REG_CONTROL_HUMIDITY, config);
}

int32_t bme280_compensate_temperature(bme280_calibration_t *calibration, int32_t raw) {
    int32_t var1, var2, T;
    
    var1 = ((((raw >> 3) - ((int32_t)calibration->dig_T1 << 1))) * 
           ((int32_t)calibration->dig_T2)) >> 11;
    
    var2 = (((((raw >> 4) - ((int32_t)calibration->dig_T1)) * 
             ((raw >> 4) - ((int32_t)calibration->dig_T1))) >> 12) * 
           ((int32_t)calibration->dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T;
}

uint32_t bme280_compensate_pressure(bme280_calibration_t *calibration, int32_t raw) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calibration->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calibration->dig_P5) << 17);
    var2 = var2 + (((int64_t)calibration->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calibration->dig_P3) >> 8) + 
           ((var1 * (int64_t)calibration->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calibration->dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0;
    }
    
    p = 1048576 - raw;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calibration->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calibration->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calibration->dig_P7) << 4);
    
    return (uint32_t)p;
}

uint32_t bme280_compensate_humidity(bme280_calibration_t *calibration, int32_t raw) {
    int32_t v_x1_u32r;
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((raw << 14) - (((int32_t)calibration->dig_H4) << 20) - 
                  (((int32_t)calibration->dig_H5) * v_x1_u32r)) + 
                 ((int32_t)16384)) >> 15) * 
                (((((((v_x1_u32r * ((int32_t)calibration->dig_H6)) >> 10) * 
                   (((v_x1_u32r * ((int32_t)calibration->dig_H3)) >> 11) + 
                   ((int32_t)32768))) >> 10) + 
                  ((int32_t)2097152)) * 
                 ((int32_t)calibration->dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
                              ((int32_t)calibration->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    return (uint32_t)(v_x1_u32r >> 12);
}

float bme280_read_temperature(void) {
    return (float) bme280_compensate_temperature(&calibration, bme280_read_raw_temperature()) / 100.0f;
}

float bme280_read_pressure(void) {
    return (float) bme280_compensate_pressure(&calibration, bme280_read_raw_pressure()) / 25600.0f;
}

float bme280_read_humidity(void) {
    return (float) bme280_compensate_humidity(&calibration, bme280_read_raw_humidity()) / 1024.0f;
}


static void app_es_task(void *param)
{
    app_print("BME280 ID: 0x%02x\r\n", bme280_read_id());
    bme280_write(BME280_REG_RESET, BME280_RESET);

    while (bme280_read_status() & BME280_STATUS_NVM_BUSY);
    app_print("BME280 NVM data ready\r\n");

    bme280_read_calibration(&calibration);

    bme280_configure(
        BME280_CONFIG_STANDBY(BME280_CONFIG_STANDBY_125) |
        BME280_CONFIG_FILTER(BME280_CONFIG_FILTER_OFF)
    );
    bme280_configure_humidity(
        BME280_MEASUREMENT_OSRH(BME280_MEASUREMENT_OSR4)
    );
    bme280_configure_measurement(
        BME280_MEASUREMENT_OSRT(BME280_MEASUREMENT_OSR4) |
        BME280_MEASUREMENT_OSRP(BME280_MEASUREMENT_OSR4) |
        BME280_MEASUREMENT_MODE_NORMAL
    );

    for (;;) {
        if (conn_idx != UINT8_MAX) {
            float temperature = bme280_read_temperature();
            float pressure = bme280_read_pressure();
            float humidity = bme280_read_humidity();
            app_print(
                "t: %d.%03d, p: %d.%03d, h: %d.%03d\r\n",
                (int) temperature, (int) (temperature * 1000.0f) % 1000,
                (int) pressure, (int) (pressure * 1000.0f) % 1000,
                (int) humidity, (int) (humidity * 1000.0f) % 1000
            );
            ble_es_srv_send_temperature(conn_idx, temperature);
            ble_es_srv_send_pressure(conn_idx, pressure);
            ble_es_srv_send_humidity(conn_idx, humidity);
        }
        sys_ms_sleep(200);
    }
}

static void app_es_config(void)
{
    rcu_periph_clock_disable(RCU_GPIOA);
    rcu_periph_clock_disable(RCU_GPIOB);
    rcu_periph_clock_disable(RCU_UART2);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_UART2);

    gpio_af_set(UART2_TX_GPIO, UART2_TX_AF_NUM, UART2_TX_PIN);
    gpio_af_set(UART2_RX_GPIO, UART2_RX_AF_NUM, UART2_RX_PIN);
    gpio_mode_set(UART2_TX_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, UART2_TX_PIN);
    gpio_output_options_set(UART2_TX_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, UART2_TX_PIN);
    gpio_mode_set(UART2_RX_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, UART2_RX_PIN);
    gpio_output_options_set(UART2_RX_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, UART2_RX_PIN);

    setvbuf(stdout, NULL, _IONBF, 0);

    usart_deinit(UART2);
    usart_word_length_set(UART2, USART_WL_8BIT);
    usart_stop_bit_set(UART2, USART_STB_1BIT);
    usart_parity_config(UART2, USART_PM_NONE);
    usart_baudrate_set(UART2, 115200U);
    usart_receive_config(UART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART2, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(UART2, USART_INT_RBNE);
    usart_receive_fifo_enable(UART2);

    usart_enable(UART2);

    while(RESET == usart_flag_get(UART2, USART_FLAG_IDLE)) {

    }

    usart_flag_clear(UART2, USART_FLAG_IDLE);
    usart_interrupt_enable(UART2, USART_INT_IDLE);

    rcu_periph_clock_enable(ES_RCU_GPIO_I2C_SCL);
    rcu_periph_clock_enable(ES_RCU_GPIO_I2C_SDA);

    gpio_af_set(ES_I2C_SCL_GPIO, ES_I2C_SCL_AF_NUM, ES_I2C_SCL_PIN);
    gpio_af_set(ES_I2C_SDA_GPIO, ES_I2C_SDA_AF_NUM, ES_I2C_SDA_PIN);
    gpio_mode_set(ES_I2C_SCL_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, ES_I2C_SCL_PIN);
    gpio_output_options_set(ES_I2C_SCL_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, ES_I2C_SCL_PIN);

    gpio_mode_set(ES_I2C_SDA_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, ES_I2C_SDA_PIN);
    gpio_output_options_set(ES_I2C_SDA_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, ES_I2C_SDA_PIN);

    rcu_periph_clock_enable(ES_RCU_I2C);

    i2c_timing_config(ES_I2CX, 7, 0x08, 0x00);
    i2c_master_clock_config(ES_I2CX, 0x30, 0x91);
    i2c_enable(ES_I2CX);
}

int app_es_init(void)
{
    if (sys_task_create_dynamic((const uint8_t *)"ES task", 512, ES_TASK_PRIORITY, app_es_task, NULL) == NULL) {
        return -1;
    }

    app_es_config();

    return 0;
}

int app_es_test(void)
{
    app_print("[ES TEST] Started\r\n");

    I2C_STAT(ES_I2CX) |= I2C_STAT_TBE;
    i2c_master_addressing(ES_I2CX, BME280_ADDRESS, I2C_MASTER_TRANSMIT);
    i2c_transfer_byte_number_config(ES_I2CX, 1);
    i2c_automatic_end_disable(ES_I2CX);

    while (i2c_flag_get(ES_I2CX, I2C_FLAG_I2CBSY));
    i2c_start_on_bus(ES_I2CX);
    while (i2c_flag_get(ES_I2CX, I2C_FLAG_TBE) != SET);

    app_print("[ES TEST] BME280 probed\r\n");

    i2c_data_transmit(ES_I2CX, BME280_REG_ID);
    while (!i2c_flag_get(ES_I2CX, I2C_FLAG_TC));

    app_print("[ES TEST] BME280 ID register request sent\r\n");

    i2c_master_addressing(ES_I2CX, BME280_ADDRESS, I2C_MASTER_RECEIVE);
    i2c_automatic_end_disable(ES_I2CX);

    i2c_start_on_bus(ES_I2CX);
    while (!i2c_flag_get(ES_I2CX, I2C_FLAG_RBNE));

    app_print("[ES TEST] BME280 ID: %02x\r\n", i2c_data_receive(ES_I2CX));
    i2c_stop_on_bus(ES_I2CX);

    return 0;
}
