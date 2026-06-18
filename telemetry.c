/*
 * telemetry.c  -  KIPR Wombat sensor dashboard data source
 *
 * Reads every port with libkipr and writes a JSON file that the
 * HTML dashboard reads. Runs in a loop until you stop the program.
 *
 * Edit the label arrays below to match what you plugged in.
 * Use "" for an empty port.
 */

#include <kipr/wombat.h>
#include <stdio.h>
#include <unistd.h>

#define TELEM_PATH "/home/pi/harrogate/public/doc/telemetry.json"
#define PERIOD_MS  200          /* how often to refresh, in milliseconds */

/* ---- Friendly names for each port. Use "" for an empty port. ---- */
static const char *motor_labels[4]  = {"Left drive", "Right drive", "Arm", "Lift"};
static const char *servo_labels[4]  = {"Claw", "Wrist", "Pan", "Tilt"};
static const char *analog_labels[6] = {"ET range", "Left light", "Right light",
                                       "Reflect", "Slider", ""};
static const char *digital_labels[10] = {"Touch L", "Touch R", "Bump F", "Bump B",
                                         "Line", "Stall", "", "", "", "Start"};

/* Write one object: {"port":N,"label":"..","<key>":V} */
static void obj_int(FILE *f, int port, const char *label,
                    const char *key, int value, int last) {
    fprintf(f, "{\"port\":%d,\"label\":\"%s\",\"%s\":%d}%s",
            port, label, key, value, last ? "" : ",");
}

/*
 * libkipr can't read back a motor's commanded PWM, so we remember it.
 * In YOUR robot code, drive motors through these wrappers instead of
 * calling motor() / off() directly, and the dashboard shows real PWM.
 */
static int last_pwm[4] = {0, 0, 0, 0};

void motor_tracked(int port, int pwm) {   /* -100..100, like motor() */
    if (port >= 0 && port < 4) last_pwm[port] = pwm;
    motor(port, pwm);
}

void off_tracked(int port) {
    if (port >= 0 && port < 4) last_pwm[port] = 0;
    off(port);
}

static void write_json(void) {
    FILE *f;
    int i;
    double pct;

    f = fopen(TELEM_PATH, "w");
    if (!f) return;

    fprintf(f, "{");

    /* Motors: pwm is the last value sent through motor_tracked(); */
    /* pos is the position counter.                                */
    fprintf(f, "\"motors\":[");
    for (i = 0; i < 4; i++) {
        fprintf(f, "{\"port\":%d,\"label\":\"%s\",\"pwm\":%d,\"pos\":%d}%s",
                i, motor_labels[i], last_pwm[i],
                (int)get_motor_position_counter(i),
                i < 3 ? "," : "");
    }
    fprintf(f, "],");

    /* Servos: get_servo_position returns 0..2047 */
    fprintf(f, "\"servos\":[");
    for (i = 0; i < 4; i++) {
        obj_int(f, i, servo_labels[i], "pos", get_servo_position(i), i == 3);
    }
    fprintf(f, "],");

    /* Analog 0..5 : 0..4095 */
    fprintf(f, "\"analog\":[");
    for (i = 0; i < 6; i++) {
        obj_int(f, i, analog_labels[i], "value", analog(i), i == 5);
    }
    fprintf(f, "],");

    /* Digital 0..9 : 0 or 1 */
    fprintf(f, "\"digital\":[");
    for (i = 0; i < 10; i++) {
        obj_int(f, i, digital_labels[i], "value", digital(i), i == 9);
    }
    fprintf(f, "],");

    /* IMU: accel scaled to g, gyro is raw signed. */
    fprintf(f,
        "\"imu\":{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%d,\"gy\":%d,\"gz\":%d},",
        accel_x() / 16384.0, accel_y() / 16384.0, accel_z() / 16384.0,
        gyro_x(), gyro_y(), gyro_z());

    /* Battery: power_level() is 0.0..1.0 */
    pct = power_level() * 100.0;
    fprintf(f, "\"battery\":{\"voltage\":%.2f,\"pct\":%d}",
            6.6 + power_level() * (8.4 - 6.6), (int)(pct + 0.5));

    fprintf(f, "}\n");
    fclose(f);
}

int main(void) {
    printf("Writing telemetry to %s every %d ms.\n", TELEM_PATH, PERIOD_MS);
    while (1) {
        write_json();
        msleep(PERIOD_MS);
    }
    return 0;
}
