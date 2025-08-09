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
/*
#define ES_I2C_SCL_GPIO   GPIOB
#define ES_I2C_SCL_PIN    GPIO_PIN_0
#define ES_I2C_SCL_AF_NUM GPIO_AF_6
#define ES_I2C_SDA_GPIO   GPIOB
#define ES_I2C_SDA_PIN    GPIO_PIN_1
#define ES_I2C_SDA_AF_NUM GPIO_AF_6
*/
#define ES_I2C_SCL_GPIO   GPIOA
#define ES_I2C_SCL_PIN    GPIO_PIN_2
#define ES_I2C_SCL_AF_NUM GPIO_AF_4
#define ES_I2C_SDA_GPIO   GPIOA
#define ES_I2C_SDA_PIN    GPIO_PIN_3
#define ES_I2C_SDA_AF_NUM GPIO_AF_4

#define BME280_ADDRESS (0x76 << 1)
#define BME280_REG_ID 0xd0
#define BME280_ID 0x60

extern uint8_t conn_idx;

static os_queue_t uart_queue;

static void app_es_task(void *param)
{
    for (;;) {
        app_print("task is running...");
        sys_ms_sleep(1000);
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
