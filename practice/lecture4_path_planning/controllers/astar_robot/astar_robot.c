/* ================================================================
 * astar_robot.c — Webots Robot Controller
 * CSC 2207 Robotics | GRP2 | Lecture 4 Practical
 * ================================================================
 *
 * WHAT THIS CONTROLLER DOES:
 *
 *  Phase 1 — Planning (Lecture 4):
 *    Runs A* on the 10×10 grid map to find a collision-free
 *    path from START cell (0,0) to GOAL cell (9,9).
 *    Prints the ASCII map with the planned path.
 *
 *  Phase 2 — Execution (Lecture 3):
 *    Drives the e-puck differential-drive robot along each
 *    waypoint using:
 *      - Encoder-based odometry  →  (x, y, θ) state estimate
 *      - Proportional heading controller (P-control)
 *        left_vel  = BASE_SPEED - Kp * heading_error
 *        right_vel = BASE_SPEED + Kp * heading_error
 *
 * ROBOT: Webots e-puck
 *   Wheel radius r  = 0.0205 m
 *   Axle length  L  = 0.053  m
 *   One grid cell   = 0.15   m  in the Webots world
 *
 * ODOMETRY (Lecture 3, slide 12):
 *   v     = (v_R + v_L) / 2
 *   omega = (v_R - v_L) / L
 *   x    += v * cos(theta) * dt
 *   y    += v * sin(theta) * dt
 *   theta += omega * dt
 * ================================================================ */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "astar.h"

/* ------------------------------------------------------------------ */
/* Robot physical parameters (e-puck)                                  */
/* ------------------------------------------------------------------ */
#define WHEEL_RADIUS     0.0205    /* metres                           */
#define AXLE_LENGTH      0.053     /* metres                           */
#define MAX_MOTOR_SPEED  6.28      /* rad/s                            */
#define BASE_SPEED       3.0       /* rad/s  — cruise speed            */
#define KP_HEADING       4.0       /* proportional gain for heading    */

/* ------------------------------------------------------------------ */
/* Grid ↔ World coordinate mapping                                     */
/* ------------------------------------------------------------------ */
#define CELL_SIZE        0.15      /* 1 grid cell = 0.15 m             */
#define GRID_ORIGIN_X   -0.675     /* world X of grid cell (0,0) centre*/
#define GRID_ORIGIN_Z   -0.675     /* world Z of grid cell (0,0) centre*/

/* arrival threshold — when we consider a waypoint reached */
#define WAYPOINT_THRESH  0.05      /* metres                           */

/* ------------------------------------------------------------------ */
/* Webots timestep                                                      */
/* ------------------------------------------------------------------ */
#define TIME_STEP        64        /* ms                               */

/* ------------------------------------------------------------------ */
/* Grid → world position                                               */
/* row maps to Z axis, col maps to X axis in Webots default frame      */
/* ------------------------------------------------------------------ */
static void cell_to_world(Cell c, double *wx, double *wz)
{
    *wx = GRID_ORIGIN_X + c.col * CELL_SIZE;
    *wz = GRID_ORIGIN_Z + c.row * CELL_SIZE;
}

/* ------------------------------------------------------------------ */
/* Angle normalisation to [-pi, pi]                                    */
/* ------------------------------------------------------------------ */
static double normalise_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* ================================================================
 * MAIN CONTROLLER
 * ================================================================ */
int main(void)
{
    /* ---- Webots init ---- */
    wb_robot_init();
    double dt = TIME_STEP / 1000.0; /* seconds per step */

    /* ---- Motors ---- */
    WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(left_motor,  INFINITY); /* velocity mode */
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor,  0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    /* ---- Wheel encoders ---- */
    WbDeviceTag left_enc  = wb_robot_get_device("left wheel sensor");
    WbDeviceTag right_enc = wb_robot_get_device("right wheel sensor");

    wb_position_sensor_enable(left_enc,  TIME_STEP);
    wb_position_sensor_enable(right_enc, TIME_STEP);

    /* ---- Odometry state: (x, y, theta) ---- */
    double odom_x     = 0.0;  /* metres                               */
    double odom_y     = 0.0;  /* metres  (mapped from world Z)        */
    double odom_theta = 0.0;  /* radians (0 = facing +X / +col)      */

    double prev_left  = 0.0;
    double prev_right = 0.0;

    /* ================================================================
     * PHASE 1 — A* PLANNING
     * ================================================================ */
    Cell start = {0, 0};
    Cell goal  = {9, 9};

    printf("\n[CONTROLLER] Running A* planner...\n");
    Path path = astar_plan(start, goal);

    if (path.length == 0) {
        printf("[CONTROLLER] ERROR: No path found — robot will stay still.\n");
        while (wb_robot_step(TIME_STEP) != -1) { /* idle */ }
        wb_robot_cleanup();
        return 1;
    }

    printf("[CONTROLLER] Path found! %d waypoints:\n", path.length);
    for (int i = 0; i < path.length; i++)
        printf("   [%2d] cell (%d, %d)\n", i, path.steps[i].row, path.steps[i].col);

    astar_print_map(&path, start, goal);

    /* ================================================================
     * PHASE 2 — ROBOT EXECUTION
     * ================================================================ */
    int wp = 1; /* skip start cell (index 0), begin driving to index 1 */

    printf("[CONTROLLER] Starting navigation. %d waypoints to follow.\n\n",
           path.length - 1);

    /* First encoder baseline */
    wb_robot_step(TIME_STEP);
    prev_left  = wb_position_sensor_get_value(left_enc);
    prev_right = wb_position_sensor_get_value(right_enc);

    /* ---- control loop ---- */
    while (wb_robot_step(TIME_STEP) != -1) {

        /* ---- 1. Read encoders ---- */
        double cur_left  = wb_position_sensor_get_value(left_enc);
        double cur_right = wb_position_sensor_get_value(right_enc);

        double dl = (cur_left  - prev_left)  * WHEEL_RADIUS;  /* arc L */
        double dr = (cur_right - prev_right) * WHEEL_RADIUS;  /* arc R */
        prev_left  = cur_left;
        prev_right = cur_right;

        /* ---- 2. Odometry update (Lecture 3 equations) ---- */
        double v     = (dr + dl) / 2.0;
        double omega = (dr - dl) / AXLE_LENGTH;

        odom_x     += v * cos(odom_theta);
        odom_y     += v * sin(odom_theta);
        odom_theta  = normalise_angle(odom_theta + omega * dt);

        /* ---- 3. Check if all waypoints done ---- */
        if (wp >= path.length) {
            printf("[CONTROLLER] Goal reached! Stopping.\n");
            printf("   Final odometry: x=%.3f m  y=%.3f m  theta=%.2f deg\n",
                   odom_x, odom_y, odom_theta * 180.0 / M_PI);
            wb_motor_set_velocity(left_motor,  0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            break;
        }

        /* ---- 4. Current target waypoint in world coords ---- */
        double tgt_x, tgt_z;
        cell_to_world(path.steps[wp], &tgt_x, &tgt_z);

        /* robot Y in Webots = odom_y, robot X in Webots = odom_x  */
        double dx   = tgt_x - odom_x;
        double dz   = tgt_z - odom_y; /* odom_y tracks the Z axis */
        double dist = sqrt(dx * dx + dz * dz);

        /* ---- 5. Waypoint arrival check ---- */
        if (dist < WAYPOINT_THRESH) {
            printf("[NAV] Reached waypoint %d  cell(%d,%d)  "
                   "odom(%.3f, %.3f, %.1f°)\n",
                   wp,
                   path.steps[wp].row, path.steps[wp].col,
                   odom_x, odom_y, odom_theta * 180.0 / M_PI);
            wp++;
            continue;
        }

        /* ---- 6. Heading controller ---- */
        double desired_heading = atan2(dz, dx);
        double heading_error   = normalise_angle(desired_heading - odom_theta);

        double left_vel  = BASE_SPEED - KP_HEADING * heading_error;
        double right_vel = BASE_SPEED + KP_HEADING * heading_error;

        /* Clamp to motor limits */
        if (left_vel  >  MAX_MOTOR_SPEED) left_vel  =  MAX_MOTOR_SPEED;
        if (left_vel  < -MAX_MOTOR_SPEED) left_vel  = -MAX_MOTOR_SPEED;
        if (right_vel >  MAX_MOTOR_SPEED) right_vel =  MAX_MOTOR_SPEED;
        if (right_vel < -MAX_MOTOR_SPEED) right_vel = -MAX_MOTOR_SPEED;

        wb_motor_set_velocity(left_motor,  left_vel);
        wb_motor_set_velocity(right_motor, right_vel);
    }

    wb_robot_cleanup();
    return 0;
}
