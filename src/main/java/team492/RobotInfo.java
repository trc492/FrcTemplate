/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import trclib.TrcPidController;
import trclib.TrcUtil;

public class RobotInfo
{
    //
    // Field dimensions in inches.
    //
    public static final double FIELD_LENGTH                     = 54*12.0;
    public static final double FIELD_WIDTH                      = 27*12.0;

    public static final double HIGH_TARGET_HEIGHT = 98;
    public static final double HIGH_VISION_TARGET_HEIGHT = TrcUtil.average(81, HIGH_TARGET_HEIGHT);
    public static final double PIVOT_HEIGHT = 15.0; // in from ground

    public static final double ROBOT_WIDTH = 18.125;
    public static final double ROBOT_LENGTH = 18.5;

    public static final double BATTERY_CAPACITY_WATT_HOUR       = 18.0*12.0;

    public static final double BATTERY_NOMINAL_VOLTAGE          = 12.0;

    //
    // Robot dimensions.
    //

    //
    // Joystick ports.
    //
    public static final int XBOX_DRIVERCONTROLLER               = 0;
    public static final int JSPORT_OPERATORSTICK                = 1;
    public static final int JSPORT_BUTTON_PANEL                 = 2;
    public static final int JSPORT_SWITCH_PANEL                 = 3;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONT_STEER = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_STEER = 4;    // 40A: Yellow
    public static final int CANID_LEFTREAR_STEER = 5;    // 40A: Green
    public static final int CANID_RIGHTREAR_STEER = 6;    // 40A: Blue

    public static final int CANID_LEFTFRONT_DRIVE = 13;    // 40A: Orange
    public static final int CANID_RIGHTFRONT_DRIVE = 14;    // 40A: Yellow
    public static final int CANID_LEFTREAR_DRIVE = 15;    // 40A: Green
    public static final int CANID_RIGHTREAR_DRIVE = 16;    // 40A: Blue

    public static final int CANID_FLYWHEEL = 7;
    public static final int CANID_SHOOTER_PITCH = 9;
    public static final int CANID_CONVEYOR = 10;
    public static final int CANID_INTAKE = 11;

    public static final int CANID_PDP = 26;
    public static final int CANID_PCM = 17;

    //
    // PDP Channels.
    //
    public static final int PDP_CHANNEL_RIGHT_BACK_WHEEL = 0;
    public static final int PDP_CHANNEL_RIGHT_FRONT_WHEEL = 3;
    public static final int PDP_CHANNEL_LEFT_FRONT_WHEEL = 12;
    public static final int PDP_CHANNEL_LEFT_BACK_WHEEL = 15;

    //
    // Pickup
    //
    // TODO: tune this garbage
    public static final double PICKUP_BLIND_PERIOD = 0.1; //seconds
    public static final double PICKUP_POWER = 1.0;
    public static final double DEPLOY_POWER = -1.0;
    public static final double[] PICKUP_CURR_THRESH = new double[] { 25.0 };
    public static final double[] DEPLOY_CURR_THRESH = new double[] { 0.7 };

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;

    //
    // Digital Input/Output ports.
    //
    public static final int CONVEYOR_PROXIMITY_SENSOR = 1;
    public static final int INTAKE_PROXIMITY_SENSOR = 2;
    public static final int LEFT_LIDAR = 3;
    public static final int RIGHT_LIDAR = 4;

    public static final int PWM_CHANNEL_LED = 0;
    public static final int NUM_LEDS = 26;

    //
    // Relay channels.
    //

    //
    // Solenoid channels.
    //
    public static final int SOL_INTAKE_EXTEND = 0;
    public static final int SOL_INTAKE_RETRACT = 1;

    //
    // Vision subsystem.
    //
    // TODO: tune this
    public static final double CAMERA_Y_OFFSET = -32.5;  // in from pivot of arm + is forward
    public static final double CAMERA_X_OFFSET = 0;    //Inches from pivot of arm to center of camera, + = right
    public static final double CAMERA_DATA_TIMEOUT              = 0.5;  //500ms
    public static final double CAMERA_CENTERED_THRESHOLD        = 2;    // +- 2 inches in x axis

    //
    // Ultrasonic sensors.
    //
    // public static final double SONAR_INCHES_PER_VOLT            = 1.0/0.0098; //9.8mV per inch
    // public static final double SONAR_ERROR_THRESHOLD            = 50.0; //value should not jump 50-in per time slice.

    public static final double LIDAR_INTER_SENSOR_DIST = 28; // TODO: tune
    public static final double LIDAR_SENSOR_Y_OFFSET = -3; // in from bumper edge. + is forward TODO: tune

    public static final double SHOOTER_BARREL_LENGTH = 30; // inches TODO: tune

    //
    // DriveBase subsystem.
    //

    public static final double STEER_DEGREES_PER_TICK = 360.0 / 4096.0;
    public static final double STEER_MAX_REQ_VEL = 1000.0; // deg/sec. max commanded velocity, not necessarily max vel
    public static final double STEER_MAX_ACCEL = 5000; // deg/sec^2
    // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
    public static final double STEER_MAX_VEL = ((18700 * 0.81 / 56.67) / 60.0) * 360.0; // deg/sec
    public static final double STEER_MAX_VEL_TICKS_PER_100MS =
        (STEER_MAX_VEL / STEER_DEGREES_PER_TICK) / 10.0; // ticks/100ms

    public static final TrcPidController.PidCoefficients magicSteerCoeff = new TrcPidController.PidCoefficients(2.0,
        0.01, 0, 1023.0 / STEER_MAX_VEL_TICKS_PER_100MS, 5.0 / STEER_DEGREES_PER_TICK);

    public static final double DRIVE_STALL_TIMEOUT              = 0.5;

    public static final double DRIVE_SLOW_XSCALE                = 0.5;
    public static final double DRIVE_SLOW_YSCALE                = 0.5;
    public static final double DRIVE_SLOW_TURNSCALE             = 0.4;

    public static final double DRIVE_MEDIUM_XSCALE              = 0.75;
    public static final double DRIVE_MEDIUM_YSCALE              = 0.75;
    public static final double DRIVE_MEDIUM_TURNSCALE           = 0.6;

    public static final double DRIVE_FAST_XSCALE                = 1.0;
    public static final double DRIVE_FAST_YSCALE                = 1.0;
    public static final double DRIVE_FAST_TURNSCALE             = 0.8;

    public static final double DRIVE_GYRO_ASSIST_KP             = 1.5;
    public static final double DRIVE_MAX_ROTATION_RATE          = 6.5;      //radians per second

    public static final double DRIVE_MAX_XPID_POWER             = 0.5;
    public static final double DRIVE_MAX_YPID_POWER             = 0.6;
    public static final double DRIVE_MAX_TURNPID_POWER          = 1.0;

    public static final double DRIVE_MAX_XPID_RAMP_RATE         = 0.5;
    public static final double DRIVE_MAX_YPID_RAMP_RATE         = 0.6;
    public static final double DRIVE_MAX_TURNPID_RAMP_RATE      = 1.0;

    public static final double ENCODER_INCHES_PER_COUNT = 2.5133;
    public static final double ENCODER_KP = 0.015;
    public static final double ENCODER_KI = 0.0;
    public static final double ENCODER_KD = 0.00;
    public static final double ENCODER_KF = 0.0;
    public static final double ENCODER_TOLERANCE = 2.0;

    public static final double ROBOT_TOP_SPEED = 192.6; // in/sec

    public static final double PURE_PURSUIT_KP = 0.02;
    public static final double PURE_PURSUIT_KI = 0.0;
    public static final double PURE_PURSUIT_KD = 0.002;
    public static final double PURE_PURSUIT_KF = 1.0 / ROBOT_TOP_SPEED;
    public static final double PURE_PURSUIT_MAX_VEL = 0.8 * ROBOT_TOP_SPEED;
    public static final double PURE_PURSUIT_MAX_ACCEL = 240; // in/sec^2

    public static final double GYRO_TURN_KP                     = 0.0055;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.00008;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;

}   // class RobotInfo
