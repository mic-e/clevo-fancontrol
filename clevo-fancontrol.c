#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define EC_SC 0x66
#define EC_DATA 0x62

#define IBF 1
#define OBF 0
#define EC_SC_READ_CMD 0x80

#define EC_REG_CPU_TEMP 0x07
#define EC_REG_GPU_TEMP 0xCD
#define EC_REG_FAN_DUTY 0xCE
#define EC_REG_FAN_RPMS_HI 0xD0
#define EC_REG_FAN_RPMS_LO 0xD1

// configuration

// times per second we poll CPU/GPU temperatures and update the fan speed
#ifndef RATE
#define RATE 5
#endif

// keep the history of temperatures for this amount of time
#ifndef TEMP_HISTORY_DURATION
#define TEMP_HISTORY_DURATION 5
#endif
// if 1, print the temperatures and fan speed info
#ifndef PRINT_STATE
#define PRINT_STATE 1
#endif

#ifndef MIN_DUTY
#define MIN_DUTY 10.0f
#endif
#ifndef MIN_DUTY_TEMP
#define MIN_DUTY_TEMP 40.0f
#endif

#ifndef MAX_DUTY
#define MAX_DUTY 100.0f
#endif
#ifndef MAX_DUTY_TEMP
#define MAX_DUTY_TEMP 70.0f
#endif

static int ec_query_cpu_temp(void);
static int ec_query_gpu_temp(void);
#if PRINT_STATE
static int ec_query_fan_duty(void);
static int ec_query_fan_rpms(void);
#endif
static void ec_write_fan_duty(int duty_percentage);
static void ec_io_wait(const uint32_t port, const uint32_t flag, const char value);
static uint8_t ec_io_read(const uint32_t port);
static void ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value);
static int temp_to_duty(int temp);
static int max(int a, int b);

#define TEMP_HISTORY_LEN (RATE * TEMP_HISTORY_DURATION)
static int temp_history[TEMP_HISTORY_LEN];
static int temp_history_pos = 0;

int max(int a, int b) { return (a > b) ? a : b; }

int main(int argc, char **argv) {
    // argc and argv are unused
    (void) argc;
    (void) argv;

    // clean the history
    for (int i = 0; i < TEMP_HISTORY_LEN; i++) { temp_history[i] = 0; }

    // map the EC IOs
    if (
        (ioperm(EC_DATA, 1, 1) != 0) || 
        (ioperm(EC_SC, 1, 1) != 0)
    ) {
        printf("could not get access to EC, got root?\n");
        return 1;
    }

    int prev_duty = -1;

    while (1) {
        // get the current temperature
        int temp_cur = max(ec_query_cpu_temp(), ec_query_gpu_temp());

        // log the temperature into the history
        temp_history[temp_history_pos++] = temp_cur;
        temp_history_pos %= TEMP_HISTORY_LEN;

        // get the highest temperature in the history
        int temp_max = 0;
        for (int i = 0; i < TEMP_HISTORY_LEN; i++) { temp_max = max(temp_max, temp_history[i]); }

        // calculate the new duty
        int new_duty = temp_to_duty(temp_max);

        #if PRINT_STATE
        printf(
            "temp=%2d°C, max: %2d°C, fan: %4drpm/%3d%% -> set %3d%%\n",
            temp_cur,
            temp_max,
            ec_query_fan_rpms(),
            ec_query_fan_duty(),
            new_duty
        );
        #endif

        // write the new duty
        if (new_duty != prev_duty) {
            ec_write_fan_duty(new_duty);
            prev_duty = new_duty;
        }

        usleep(1000 * 1000 / RATE);
    }
}

int temp_to_duty(int temp) {
    // scale the temperature range down to the given limits
    float x = ((float)temp - MIN_DUTY_TEMP) / (MAX_DUTY_TEMP - MIN_DUTY_TEMP);
    // ensure that x is in the proper scaling region
    if (x < 0) { x = 0; }
    if (x > 1) { x = 1; }
    // map temperature range 0..1 to duty cycle range 0..1
    const float linear_component = 0.2f;
    float y = x * (x + linear_component) / (1 + linear_component);
    // scale duty cycles back up to the given limits
    return y * (MAX_DUTY - MIN_DUTY) + MIN_DUTY;

    // this curve is designed to closely match the one from tuxedo-fan-control,
    // see fanfit.py
}

static int ec_query_cpu_temp(void) {
    return ec_io_read(EC_REG_CPU_TEMP);
}

static int ec_query_gpu_temp(void) {
    return ec_io_read(EC_REG_GPU_TEMP);
}

#if PRINT_STATE
static int ec_query_fan_duty(void) {
    int raw_duty = ec_io_read(EC_REG_FAN_DUTY);
    return (int) ((float) raw_duty / 255.0f * 100.0f);
}

static int ec_query_fan_rpms(void) {
    int raw_rpm_hi = ec_io_read(EC_REG_FAN_RPMS_HI);
    int raw_rpm_lo = ec_io_read(EC_REG_FAN_RPMS_LO);

    int raw_rpm = (raw_rpm_hi << 8) + raw_rpm_lo;
    return raw_rpm > 0 ? (2156220 / raw_rpm) : 0;
}
#endif

static void ec_write_fan_duty(int duty_percentage) {
    // sanitize input
    if (duty_percentage < 10) { duty_percentage = 10; }
    if (duty_percentage > 100) { duty_percentage = 100; }

    int v_i = (int)(((float) duty_percentage) / 100.0f * 255.0f);
    ec_io_do(0x99, 0x01, v_i);
}

static void ec_io_wait(const uint32_t port, const uint32_t flag,
        const char value) {
    uint8_t data = inb(port);
    int i = 0;
    while ((((data >> flag) & 0x1) != value) && (i++ < 100)) {
        usleep(1000);
        data = inb(port);
    }
    if (i >= 100) {
        printf("wait_ec error on port 0x%x, data=0x%x, flag=0x%x, value=0x%x\n",
                port, data, flag, value);
    }
}

static uint8_t ec_io_read(const uint32_t port) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(EC_SC_READ_CMD, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    //wait_ec(EC_SC, EC_SC_IBF_FREE);
    ec_io_wait(EC_SC, OBF, 1);
    return inb(EC_DATA);
}

static void ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(cmd, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    ec_io_wait(EC_SC, IBF, 0);
    outb(value, EC_DATA);

    ec_io_wait(EC_SC, IBF, 0);
}

