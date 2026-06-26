#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// ======================================================
// UART CONFIG
// ======================================================

#define UART_PORT UART_NUM_1

#define TX_PIN 19
#define RX_PIN 21

#define BUF_SIZE 256

// ======================================================
// PWM GPIOs
// ======================================================

#define UH_GPIO 27
#define VH_GPIO 25
#define WH_GPIO 32

// ======================================================
// PWM CONFIG
// ======================================================

#define PWM_FREQ       32000
#define PWM_MAX_DUTY   255

// ======================================================
// UART INIT
// ======================================================

void uart_init_kitsat()
{
    uart_config_t uart_config = {};

    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install(
        UART_PORT,
        BUF_SIZE * 2,
        0,
        0,
        NULL,
        0
    );

    uart_param_config(
        UART_PORT,
        &uart_config
    );

    uart_set_pin(
        UART_PORT,
        TX_PIN,
        RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
}

// ======================================================
// PWM INIT
// ======================================================

void pwm_init()
{
    // ----------------------------------------------
    // TIMER
    // ----------------------------------------------

    ledc_timer_config_t timer = {};

    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.freq_hz = PWM_FREQ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    ledc_timer_config(&timer);

    // ----------------------------------------------
    // UH
    // ----------------------------------------------

    ledc_channel_config_t uh = {};

    uh.gpio_num = UH_GPIO;
    uh.speed_mode = LEDC_LOW_SPEED_MODE;
    uh.channel = LEDC_CHANNEL_0;
    uh.intr_type = LEDC_INTR_DISABLE;
    uh.timer_sel = LEDC_TIMER_0;
    uh.duty = 0;
    uh.hpoint = 0;

    ledc_channel_config(&uh);

    // ----------------------------------------------
    // VH
    // ----------------------------------------------

    ledc_channel_config_t vh = {};

    vh.gpio_num = VH_GPIO;
    vh.speed_mode = LEDC_LOW_SPEED_MODE;
    vh.channel = LEDC_CHANNEL_1;
    vh.intr_type = LEDC_INTR_DISABLE;
    vh.timer_sel = LEDC_TIMER_0;
    vh.duty = 0;
    vh.hpoint = 0;

    ledc_channel_config(&vh);

    // ----------------------------------------------
    // WH
    // ----------------------------------------------

    ledc_channel_config_t wh = {};

    wh.gpio_num = WH_GPIO;
    wh.speed_mode = LEDC_LOW_SPEED_MODE;
    wh.channel = LEDC_CHANNEL_2;
    wh.intr_type = LEDC_INTR_DISABLE;
    wh.timer_sel = LEDC_TIMER_0;
    wh.duty = 0;
    wh.hpoint = 0;

    ledc_channel_config(&wh);
}

// ======================================================
// SET PWM DUTIES
// ======================================================

void set_pwm(
    uint32_t uh,
    uint32_t vh,
    uint32_t wh
)
{
    // UH

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        uh
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0
    );

    // VH

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1,
        vh
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1
    );

    // WH

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_2,
        wh
    );

    ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_2
    );
}

// ======================================================
// FNV CHECKSUM
// ======================================================

uint32_t ufnv(char *bytes, int str_len)
{
    uint32_t hval = 0x811c9dc5;
    uint32_t fnv_32_prime = 0x01000193;
    uint64_t uint32_max = 4294967296;

    for(int i = 0; i < str_len; i++)
    {
        hval = hval ^ bytes[i];
        hval = (hval * fnv_32_prime) % uint32_max;
    }

    return hval;
}

// ======================================================
// CREATE KITSAT COMMAND
// ======================================================

uint8_t createKitsatCommand(
    char *buf,
    uint8_t subsystem_id,
    uint8_t command_id,
    char *parameters,
    uint8_t parameters_len
)
{
    buf[0] = subsystem_id;
    buf[1] = command_id;
    buf[2] = parameters_len;

    memcpy(
        buf + 3,
        parameters,
        parameters_len
    );

    uint32_t fnv =
        ufnv(buf, parameters_len + 3);

    memcpy(
        buf + 3 + parameters_len,
        &fnv,
        4
    );

    return parameters_len + 7;
}

// ======================================================
// TELEMETRY STRUCT
// ======================================================

struct telemetry
{
    uint8_t target_id;
    uint8_t command_id;
    uint32_t timestamp;
    uint8_t data_length;
    char *data;
    uint8_t valid_telemetry = 0;
};

// ======================================================
// PARSE TELEMETRY
// ======================================================

struct telemetry parse_telemetry(
    char *buf,
    size_t len
)
{
    struct telemetry tm;

    tm.valid_telemetry = 0;
    tm.data = nullptr;

    if(len < 11)
        return tm;

    tm.target_id = buf[0];
    tm.command_id = buf[1];

    tm.timestamp =
        ((uint32_t)buf[2] << 24) |
        ((uint32_t)buf[3] << 16) |
        ((uint32_t)buf[4] << 8)  |
        ((uint32_t)buf[5]);

    tm.data_length = buf[6];

    if(len < 7 + tm.data_length + 4)
        return tm;

    tm.data =
        (char*)malloc(tm.data_length + 1);

    if(tm.data == nullptr)
        return tm;

    memcpy(
        tm.data,
        &buf[7],
        tm.data_length
    );

    tm.data[tm.data_length] = '\0';

    uint32_t received_checksum =
        ((uint32_t)buf[tm.data_length + 10] << 24) |
        ((uint32_t)buf[tm.data_length + 9] << 16) |
        ((uint32_t)buf[tm.data_length + 8] << 8) |
        ((uint32_t)buf[tm.data_length + 7]);

    uint32_t calculated_checksum =
        ufnv(buf, tm.data_length + 7);

    if(received_checksum ==
       calculated_checksum)
    {
        tm.valid_telemetry = 1;
    }
    else
    {
        free(tm.data);
        tm.data = nullptr;
    }

    return tm;
}

// ======================================================
// MAIN
// ======================================================

extern "C" void app_main(void)
{
    // ----------------------------------------------
    // INIT
    // ----------------------------------------------

    uart_init_kitsat();

    pwm_init();

    printf("System started\n");

    // ----------------------------------------------
    // COMMAND BUFFER
    // ----------------------------------------------

    char cmd[64];

    // comando gyro

    uint8_t len =
        createKitsatCommand(
            cmd,
            5,
            5,
            (char*)"100",
            0
        );

    // ----------------------------------------------
    // COMMUTATION STEP
    // ----------------------------------------------

    int step = 0;

    // ----------------------------------------------
    // MAIN LOOP
    // ----------------------------------------------

    while(1)
    {
        // ------------------------------------------
        // SEND COMMAND
        // ------------------------------------------

        uart_write_bytes(
            UART_PORT,
            cmd,
            len
        );

        // ------------------------------------------
        // RECEIVE UART
        // ------------------------------------------

        uint8_t rx_buffer[BUF_SIZE];

        int rx_len =
            uart_read_bytes(
                UART_PORT,
                rx_buffer,
                BUF_SIZE,
                pdMS_TO_TICKS(100)
            );

        // ------------------------------------------
        // VALID DATA
        // ------------------------------------------

        if(rx_len > 0)
        {
            struct telemetry tm =
                parse_telemetry(
                    (char*)rx_buffer,
                    rx_len
                );

            if(tm.valid_telemetry)
            {
                // ==================================
                // GYRO VALUE
                // ==================================

                float gyro_x =
                    atof(tm.data);

                // ==================================
                // PHYSICAL PARAMETERS
                // ==================================

                float I_sat = 0.02f;
                float I_rw  = 0.0002f;

                float stop_time = 0.5f;

                // ==================================
                // SATELLITE ALPHA
                // ==================================

                float alpha_sat =
                    -gyro_x / stop_time;

                // ==================================
                // REQUIRED TORQUE
                // ==================================

                float torque =
                    I_sat * alpha_sat;

                // ==================================
                // RW ALPHA
                // ==================================

                float alpha_rw =
                    -torque / I_rw;

                // ==================================
                // CONVERT TO DUTY
                // ==================================

                int duty =
                    (int)(fabs(alpha_rw)*100.0f);

                if(duty > PWM_MAX_DUTY)
                    duty = PWM_MAX_DUTY;

                // ==================================
                // PRINT RESULTS
                // ==================================

                printf("\n");
                printf("====================================\n");

                printf("gyro_x = %.3f rad/s\n",
                       gyro_x);

                printf("alpha_rw = %.3f rad/s^2\n",
                       alpha_rw);

                printf("PWM duty = %d\n",
                       duty);

                printf("commutation step = %d\n",
                       step);

                printf("PWM = %.1f %%\n",
                        100.0f * duty / 255.0f);       

                printf("====================================\n");
                printf("\n");

                // ==================================
                // THREE PHASE SEQUENCE
                // ==================================

                switch(step)
                {
                    case 0:

                        // U active

                        set_pwm(
                            duty,
                            0,
                            0
                        );

                        break;

                    case 1:

                        // V active

                        set_pwm(
                            0,
                            duty,
                            0
                        );

                        break;

                    case 2:

                        // W active

                        set_pwm(
                            0,
                            0,
                            duty
                        );

                        break;
                }

                // ==================================
                // NEXT STEP
                // ==================================

                step++;

                if(step > 2)
                    step = 0;

                // ==================================
                // FREE MEMORY
                // ==================================

                free(tm.data);
            }
            else
            {
                printf("Invalid telemetry\n");
            }
        }

        // ------------------------------------------
        // COMMUTATION SPEED
        // ------------------------------------------

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
