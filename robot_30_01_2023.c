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
float ball_distance = 0;
int lines_detected = 0;
int wall_detected = 0;
bool ball_detected = false;
int turn_to_ball = 0;

/// @brief Initialization of motors, sensors etc
/// @authors: Maryam
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
        printf("ERROR: motor is not found\n");
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
    else
    {
        return (1);
    }

    return (0);
}

/// @brief Function for running 2 motors indefinitly
/// @param l_speed the input speed of the left wheel
/// @param r_speed the input speed of the right wheel
/// @authors Alberto, Lenia
static void run_forever(int l_speed, int r_speed)
{
    printf("run_forever()... \n");

    set_tacho_speed_sp(sn_left, l_speed * 0.2);
    set_tacho_speed_sp(sn_right, r_speed * 0.2);
    set_tacho_command_inx(sn_left, TACHO_RUN_FOREVER);
    set_tacho_command_inx(sn_right, TACHO_RUN_FOREVER);
}

/// @brief Stops both wheels at the same time by resetting the motors
/// @authors Alberto, Lenia
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
/// @authors Ludovic, Lenia
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
/// @authors Ludovic, Alberto
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
/// @authors Ludovic, Lenia
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

        printf("OLD VALUE: %f \n", old_value);
        old_value = new_value;
        get_sensor_value0(sensor_sonar, &new_value);
        printf("NEW VALUE = %f \n", new_value);

        // If the ball has been detected
        if (new_value - old_value > 20 || old_value - new_value > 20 || new_value > 1000)
        {
            // check_line = 1;
            printf("---BALL DETECTED-----\n");
            printf("NEW VALUE = %f \n", new_value);
            ball_distance = new_value;
            ball_detected = true;
            stop_run();
            return;
        }

        // Scan 15 times to the left
        if (counter <= 15)
        {
            printf("left \n");
            run_timed(0, speed_motor, 50);
        }

        // Scan to the right
        else
        {
            printf("right \n");
            run_timed(speed_motor, 0, 50);
        }

        sleep(1);
    }
}

/// @brief Controls the claw by opening and closing it
/// @param open True if we want to open the claw
/// @authors Ludovic, Alberto
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
/// @authorss Alberto, Lenia
void throw()  
{
   // move_motor(sn_left, 1200, (float)1 / 2);
    printf("throw()... \n");
    sleep(2);
    move_motor(sn_arm, 630, -1); // Move arm up
    sleep(1);
    // open_claw(true);
}

/// @brief Function for controlling the clas in order to replicate a grabbing movement
/// @authors Ludovic, Alberto 
void grab()
{
    int speed_motor;
    get_tacho_max_speed(sn_right, &speed_motor);

    printf("grab()... \n");
    // open_claw(true);
    // open_claw(true);
    //    sleep(1);
    //   move_motor(sn_left, 80, 1); // Rotate right to position arm above ball
    //   sleep(1);
    //   move_motor(sn_arm, 600, (float)1 / 9); // Take arm down
    sleep(1);
    run_timed(speed_motor, speed_motor, 100);
    open_claw(false); // Close claw
}

/// @brief Used to scan for spherical objects and grab it if it has been detected
/// @authors Ludovic, Lenia
void detect_and_fetch()
{
    printf("detect_and_fetch()... \n");
    detect_ball();

    // Once ball is detected
    if (ball_detected)
    {
        float temp_distance = 300.5;
        int speed_motor;
        get_tacho_max_speed(sn_right, &speed_motor);
        run_timed(-speed_motor, -speed_motor, 2000); // Move  back

        printf("ball_distance: %f \n", ball_distance);
        sleep(2);
        run_timed(speed_motor, 0, 75); // Rotate right to position arm above ball
        sleep(2);
        open_claw(true);
        move_motor(sn_arm, 600, (float)1 / 9); // Move arm down
        sleep(1);

        // Go straight until the distance is enough close
        run_forever(speed_motor, speed_motor);
        while (temp_distance >= 230)
        {
            printf("checking distance \n");
            get_sensor_value0(sensor_sonar, &temp_distance);
        }
        // start grabbing the ball
        stop_run();
        sleep(0.2);
        grab();
        printf("final position \n");
        // stopping to run
        sleep(1);

        // Go back in order to avoid touching the wall and be blocked
        run_timed(-speed_motor, -speed_motor, 600); // go back

        //        throw();
    }
    ball_detected = false;
}

/// @brief 
/// @authors Ludovic, Alberto
static void defender_strategy()
{
    int speed_motor;
    float turn_distance = 300;
    int stop_color = 5;

    int white_black = 0;

    get_tacho_max_speed(sn_right, &speed_motor);
    // Go straight until the middle
    run_forever(speed_motor, speed_motor);
    int black_lines = 0;
    while (true)
    {
        // sleep(0.5);
        get_sensor_value(0, sensor_color, &stop_color);
        printf("color  IS %d \n", stop_color);

        if (stop_color <= 3)
        {
            white_black = 1;
            black_lines++;
            sleep(2);
        }
        if (stop_color > 3)
        {
            white_black = 0;
        }
        if (black_lines == 1)
        {
            break;
        }
    }
    stop_run();
    sleep(2);

    // turn right
    run_timed(speed_motor, -speed_motor, 520);
    sleep(1);

    // go straight until the wall is detected
    run_forever(speed_motor, speed_motor);
    sleep(1);
    printf("BEfore while: %f", turn_distance);
    while (turn_distance > 200)
    {
        printf("TURN DISTANCE IS %f \n", turn_distance);
        get_sensor_value0(sensor_sonar, &turn_distance);
    }
    stop_run();
    sleep(1);
    // Turn right
    run_timed(speed_motor, -speed_motor, 520);
    sleep(1);
    turn_distance = 500;

    // Go straight until the wall is detected
    run_forever(speed_motor, speed_motor);

    while (turn_distance > 200)
    {
        //            printf("TURN DISTANCE IS %f \n",turn_distance);
        get_sensor_value0(sensor_sonar, &turn_distance);
    }
    stop_run();
    sleep(1);
    printf("SHOULD TURN \n");
    run_timed(speed_motor, -speed_motor, 520);
    sleep(2);

    // From now, the robot will continously going straight and back to prevent the adversary scoring
    while (true)
    {
        printf("Infinite loop \n");
        turn_distance = 400;
        run_forever(speed_motor, speed_motor);
        // sleep(2);
        while (turn_distance > 300)
        {
            printf("TURN DISTANCE IS %f \n", turn_distance);
            get_sensor_value0(sensor_sonar, &turn_distance);
        }
        sleep(1);
        stop_run();
        printf("last rotation\n");
        run_timed(speed_motor, 0, 1200);
        sleep(1.5);
    }

    return;
}

/// @authors Ludovic, Lenia
CORO_DEFINE(coro_detect_line)
{
    CORO_BEGIN();

    printf("coro_detect_line()... \n");
    for (;;)
    {
//      sleep(2);
     if (!get_sensor_value(0, sensor_color, &color_value) || (color_value < 0) || (color_value >= COLOR_COUNT))
        {
            color_value = 0;
        }
        printf("color is %d\n",color_value);
        if (color_value < 3) // Black line detected
        {
            printf("Black line detected... stop. \n");
            //stop_run();
            sleep(1.5);
            lines_detected++;
        }

        if (lines_detected == 2)
        {
        stop_run();
            turn_to_ball = 1;
        }

        CORO_YIELD();
    }
}

/// @brief For detecting obstacles that are too close using sonar.
/// @authors Alberto, Lenia, Ludovic
CORO_DEFINE(coro_detect_wall)
{
    CORO_BEGIN();
    printf("coro_detect_wall()... \n");
    for (;;)
    {
        get_sensor_value0(sensor_sonar, &distance);
        if (distance < 300)
        {
            wall_detected++;
            printf("STOP! Distance is: %f \n", distance);
            stop_run();
            sleep(1);
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
/// @authors Alberto, Lenia
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

/// @authorss Alberto, Lenia, Ludovic 
int main(void)
{
    int defender = 0;
    printf("Waiting the EV3 brick online...\n");
    if (ev3_init() < 1)
        return (1);
    printf("*** ( EV3 ) Hello! ***\n");
    ev3_sensor_init();
    ev3_tacho_init();
    app_alive = app_init();
    printf("app_alive = %d\n", app_alive);
    int cc = 0;
    open_claw(false);
    open_claw(false);
    /* open_claw(true);
     sleep(3);
     open_claw(false);
     throw();
     sleep(5);
     if(defender==1){
         defender_strategy();
     }*/

    while (app_alive == 0)
    {

        // Starting position --> run & start detect lines
        if (lines_detected < 2)
        {
            printf("Starting detecting line \n");
            CORO_CALL(coro_drive_forever);
            CORO_CALL(coro_detect_line);
       }
        // When we  detect x lines, we have detected the second line (we are in the middle);
        if (lines_detected == 2 && wall_detected == 0)
        {
            // Before starting detecting the wall we turn
            if (turn_to_ball == 1)
            {
                move_motor(sn_right, 530,  1); // Turn left
                sleep(2);
                turn_to_ball = 0;
            }
            printf("Turning and detecting the wall \n");
            sleep(1);
            // RUN until the wall is detcted
            CORO_CALL(coro_drive_forever);
            CORO_CALL(coro_detect_wall);
        }

        // When we detect the wall,
        if (wall_detected == 1)
        {

            printf("Detecting the ball \n");
            // detect the ball and grabb it
            detect_and_fetch();
            printf("STOP THE APPLICATION \n");
             int max_speed_L;
        int max_speed_R;
        get_tacho_max_speed(sn_left, &max_speed_L);
        get_tacho_max_speed(sn_right, &max_speed_R);
        run_timed(-max_speed_L,0,600);
        sleep(1);
        run_timed(-max_speed_L,-max_speed_R,200);
        throw();
            sleep(100);
        }
        //      if(wall_detected==2){
        //        detect_and_fetch();
        //        printf("STOP APPLICATION \n");
        //        sleep(100);
        //      }

        //     detect_ball();
        //   sleep(10);
        /*
                if(wall_detected==0){
                CORO_CALL(coro_drive_forever);
                CORO_CALL(coro_detect_wall);
                }
                if(wall_detected==1 && cc==0){
                detect_and_fetch();
                cc++;}
        */
    }
    ev3_uninit();
    printf("*** ( EV3 ) Bye! ***\n");
    return (0);
}
