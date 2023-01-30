#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep(msec) usleep((msec)*1000)

//////////////////////////////////////////////////
#endif

////////////////// COROUTINE CONTEXTS ///////////////////////////////
CORO_CONTEXT(coro_detect_line);
CORO_CONTEXT(coro_detect_wall);
CORO_CONTEXT(coro_drive_forever);
/////////////////////////////////////////////////////////////////////

const char const *color[] = {"?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN"};
#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

uint8_t sn_left, sn_right, sn_claw, sn_arm;
uint8_t sensor_sonar, sensor_color;
int app_alive;
int val;
float distance;
int color_value;

int lines_detected = 0;
int wall_detected = 0;
bool ball_detected = false;

// Author: Maryam
/// @brief Initialization of motors, sensors etc
int app_init(void)
{
    printf("app_init() ... \n");
    uint32_t n, ii;
    int i;
    char s[256];
    for (i = 0; i < DESC_LIMIT; i++)
    {
        if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_)
        {
            printf("  type = %s\n", ev3_tacho_type(ev3_tacho[i].type_inx));
            printf("  port = %s\n", ev3_tacho_port_name(i, s));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }

    if (!(ev3_search_tacho_plugged_in(65, 0, &sn_left, 0) && ev3_search_tacho_plugged_in(66, 0, &sn_arm, 0) && ev3_search_tacho_plugged_in(67, 0, &sn_claw, 0) && ev3_search_tacho_plugged_in(68, 0, &sn_right, 0)))
    {
        printf("ERROR: A motor is not found\n");
        return (1);
    }

    // ev3_sensor_init();

    printf("Found sensors:\n");
    for (i = 0; i < DESC_LIMIT; i++)
    {
        if (ev3_sensor[i].type_inx != SENSOR_TYPE__NONE_)
        {
            printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
            printf("  port = %s\n", ev3_sensor_port_name(i, s));
            if (get_sensor_mode(i, s, sizeof(s)))
            {
                printf("  mode = %s\n", s);
            }
            if (get_sensor_num_values(i, &n))
            {
                for (ii = 0; ii < n; ii++)
                {
                    if (get_sensor_value(ii, i, &val))
                    {
                        printf("  value%d = %d\n", ii, val);
                    }
                }
            }
        }
    }

    // Initialize the ultrasonic sensor (SONAR)
    if (!ev3_search_sensor(LEGO_EV3_US, &sensor_sonar, 0))
    {
        printf("SONAR sensor is not found\n");
        fflush(stdout);
        return (1);
    }

    if (ev3_search_sensor(LEGO_EV3_COLOR, &sensor_color, 0))
    {
        printf("COLOR sensor is found, reading COLOR...\n");
        set_sensor_mode(sensor_color, "COL-COLOR");

        fflush(stdout);
    }

    return (1);
}

/// @brief Function for running 2 motors indefinitly 
/// @param l_speed the input speed of the left wheel 
/// @param r_speed the input speed of the right wheel 
static void run_forever(int l_speed, int r_speed)
{
    printf("run_forever()... \n");

    set_tacho_speed_sp(sn_left, l_speed * 0.3);
    set_tacho_speed_sp(sn_right, r_speed * 0.3);
    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
}

/// @brief Stops both wheels at the same time by resetting the motors
static void stop_run()
{
    printf("stop_run()... \n");

    set_tacho_command_inx(sn_left, TACHO_RESET);
    set_tacho_command_inx(sn_right, TACHO_RESET);
}

/// @brief Function for running 2 motors for a given time period  
/// @param l_speed The speed of the left motor
/// @param r_speed The speed of the right motor
/// @param time The time period in which the motors should run
static void run_timed(int l_speed, int r_speed, int time)
{
    printf("run_timed()... \n");

    set_tacho_speed_sp(sn_left, l_speed * 0.3);
    set_tacho_speed_sp(sn_right, r_speed * 0.3);
    set_tacho_time_sp(sn_right, time);
    set_tacho_time_sp(sn_left, time);
    set_tacho_command_inx(sn_left, TACHO_RUN_TIMED);
    set_tacho_command_inx(sn_right, TACHO_RUN_TIMED);
}

/// @brief Function to control a single motor given a time period
/// @param sn the port of the motor
/// @param time the time in which the motor should run
/// @param speed_perc Specifies the direction of the motor
/// @return returns a simple int
int move_motor(uint8_t sn, int time, float speed_perc)
{
    printf("move_motor()... \n");

    int max_speed;
    FLAGS_T state;

    get_tacho_max_speed(sn, &max_speed);
    set_tacho_stop_action_inx(sn, TACHO_BRAKE);
    set_tacho_speed_sp(sn, max_speed * speed_perc);
    set_tacho_time_sp(sn, time);
    set_tacho_command_inx(sn, TACHO_RUN_TIMED);

    while (get_tacho_state_flags(sn, &state) && state)
    {
        if (state == 2)
        {
            set_tacho_command_inx(sn, TACHO_RESET);
            printf("___ MOTOR STALLED____\n");
            break;
        }
    }

    return 0;
}

/// @brief Function for detecting spherical objects by checking for a drop in distande measured while scanning. 
void detect_ball()
{
    printf("detect_ball()... \n");
    int speed_motor;
    get_tacho_max_speed(sn_right, &speed_motor);
    int counter = 0;
    float new_value;
    float old_value;
    get_sensor_value0(sensor_sonar, &new_value);
    get_sensor_value0(sensor_sonar, &old_value);
    while (true)
    {
        counter++;

        // printf("OLD VALUE: %f \n", old_value);
        old_value = new_value;
        get_sensor_value0(sensor_sonar, &new_value);
        // printf("NEW VALUE = %f \n", new_value);

        // If the ball has been detected
        if (new_value - old_value > 30 || old_value - new_value > 30 || new_value > 1000)
        {
            // check_line = 1;
            printf("---BALL DETECTED-----\n");
            printf("NEW VALUE = %f \n", new_value);
            ball_detected = true;
            stop_run();
            return;
        }

        // Scan 15 times to the left
        if (counter <= 15)
        {
            run_timed(0, speed_motor, 50);
        }

        // Scan to the right
        else
        {
            run_timed(speed_motor, 0, 50);
        }
    }
}

/// @brief Controls the claw by opening and closing it
/// @param open True if we want to open the claw
void open_claw(bool open)
{
    printf("open_claw()... \n");
    if (open)
    {
        move_motor(sn_claw, 900, 1); // Open
    }
    else
    {
        move_motor(sn_claw, 900, -1); // Close
    }
}

/// @brief Function for moving the arm up and down in order to replicate a throwing movement
void throw()
{
    printf("throw()... \n");
    move_motor(sn_arm, 630, -1); // Move arm up
    sleep(1);
    open_claw(true);
}

/// @brief Function for controlling the clas in order to replicate a grabbing movement
void grab()
{
    printf("grab()... \n");
    open_claw(true);
    sleep(1);
    move_motor(sn_left, 500, 1); // Rotate right to position arm above ball
    sleep(1);
    move_motor(sn_arm, 600, (float)1 / 9); // Take arm down
    open_claw(false);                      // Close claw
}

/// @brief Used to scan for spherical objects and grab it if it has been detected
void detect_and_fetch()
{
    printf("detect_and_fetch()... \n");
    detect_ball();
    if (ball_detected)
    {
        grab(); // Makes us rotate right to position arm above ball
        sleep(1);
        throw();
    }
    ball_detected = false;
}

/// @brief For detecting black lines with color sensor
CORO_DEFINE(coro_detect_line)
{
    CORO_BEGIN();
    printf("coro_detect_line()... \n");
    for (;;)
    {
        if (!get_sensor_value(0, sensor_color, &color_value) || (color_value < 0) || (color_value >= COLOR_COUNT))
        {
            color_value = 0;
        }

        if (color_value < 3) // Black line detected
        {
            lines_detected++;
            drive = false; // Stop driving forawrd

            printf("Black line detected... stop. \n");
            line_detected = true;
            stop_run();
            move_motor(sn_right, 580, 1); // Turn left
        }

        CORO_YIELD();
    }
}

/// @brief For detecting obstacles that are too close. 
CORO_DEFINE(coro_detect_wall)
{
    CORO_BEGIN();
    printf("coro_detect_wall()... \n");
    for (;;)
    {
        get_sensor_value0(sensor_sonar, &distance);
        if (distance < 200)
        {
            wall_detected = 1;
            printf("STOP! Distance is: %f \n", distance);
            stop_run();
        }
        else
        {
            printf("Don't stop. Distance is %f \n", distance);
        }
        CORO_YIELD();
    }
    CORO_END();
}

/// @brief For running both wheels forever
CORO_DEFINE(coro_drive_forever)
{

    CORO_BEGIN();
    printf("coro_drive_forever()... \n");
    for (;;)
    {
        int max_speed_L;
        int max_speed_R;
        get_tacho_max_speed(sn_left, &max_speed_L);
        get_tacho_max_speed(sn_right, &max_speed_R);
        run_forever(max_speed_L, max_speed_R);
        CORO_YIELD();
    }
    CORO_END();
}

int main(void)
{
    printf("Waiting the EV3 brick online...\n");
    if (ev3_init() < 1)
        return (1);
    printf("*** ( EV3 ) Hello! ***\n");
    ev3_sensor_init();
    ev3_tacho_init();
    app_alive = app_init();

    while (app_alive)
    {
        if (lines_detected == 0)
        {
            CORO_CALL(coro_drive_forever);
            CORO_CALL(coro_detect_line);
        }
        if (lines_detected == 1 && wall_detected == 0)
        {
            CORO_CALL(coro_drive_forever);
            CORO_CALL(coro_detect_wall);
        }

        if (wall_detected == 1)
        {
            printf("wall_detected == 1 \n");
            detect_and_fetch();
        }
    }
    ev3_uninit();
    printf("*** ( EV3 ) Bye! ***\n");
    return (0);
}
