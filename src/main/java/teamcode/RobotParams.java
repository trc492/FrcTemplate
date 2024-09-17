/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.drivebase.FrcRobotDrive.GyroType;
import frclib.drivebase.FrcSwerveDrive.SteerEncoderType;
import frclib.motor.FrcMotorActuator.MotorType;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.drivebase.TrcDriveBase.OdometryType;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcPidController.PidCoefficients;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains field dimension constants. Generally, these should not change. But some seasons may have
     * slight variations of the field dimensions.
     */
    public static class Field
    {
        // Field dimensions in inches.
        public static final double LENGTH                       = 54.0*12.0;
        public static final double WIDTH                        = 27.0*12.0;
    }   //class Field

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        //
        // Robot starting positions.
        //
        public static final double STARTPOS_BLUE_Y              = Robot.ROBOT_LENGTH / 2.0;
        public static final double STARTPOS_RED_Y               = Field.LENGTH - STARTPOS_BLUE_Y;
        public static final double STARTPOS_1_X                 = -42.19;
        public static final double STARTPOS_2_X                 = -108.19;
        public static final double STARTPOS_3_X                 = -174.19;
        public static final TrcPose2D STARTPOS_BLUE_1           = new TrcPose2D(STARTPOS_1_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_2           = new TrcPose2D(STARTPOS_2_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_3           = new TrcPose2D(STARTPOS_3_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_RED_1            = new TrcPose2D(STARTPOS_1_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D STARTPOS_RED_2            = new TrcPose2D(STARTPOS_2_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D STARTPOS_RED_3            = new TrcPose2D(STARTPOS_3_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D[][] startPoses            =
        {
            {STARTPOS_BLUE_1, STARTPOS_BLUE_2, STARTPOS_BLUE_3},
            {STARTPOS_RED_1, STARTPOS_RED_2, STARTPOS_RED_3}
        };
        //
        // Game element locations and dimensions.
        //
    }   //class Game

    /**
     * This class contains Robot parameters.
     */
     public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             = "/home/lvuser/trc492";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final String FIELD_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/FieldZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.1;      // in msec
        public static final String ROBOT_CODEBASE               = "Robot2025";
        public static final double ROBOT_LENGTH                 = 34.5;
        public static final double ROBOT_WIDTH                  = 34.5;
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
        public static final double DRIVE_RAMP_RATE              = 0.25;
    }   //class Robot
 
    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Control Hub and camera.
        VisionOnly,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Swerve Drive Base Robot
        SwerveRobot
    }   //enum RobotType

    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.SwerveRobot;
        public static final boolean inCompetition               = false;
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useCommStatusMonitor        = true;
        // Status Update
        public static final boolean doStatusUpdate              = true;
        public static final boolean showLoopTime                = false;
        public static final boolean showPowerConsumption        = false;
        public static final boolean showDriveBase               = false;
        public static final boolean showPurePursuitDrive        = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showVision                  = false;
        public static final boolean showSubsystems              = true;
        // Inputs
        public static final boolean useDriverXboxController     = true;
        public static final boolean useOperatorXboxController   = true;
        public static final boolean useTankDrive                = false;
        public static final boolean doOneStickDrive             = false;
        public static final boolean useButtonPanels             = false;
        // Sensors
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        public static final boolean usePressureSensor           = false;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean usePhotonVision             = true;
        public static final boolean usePhotonVisionRaw          = false;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        // Drive Base
        public static final boolean useDriveBase                = false;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        public static final boolean useSteeringCANCoder         = false;
        public static final boolean useSteeringCanandmag        = true;
        public static final boolean useSteeringAnalogEncoder    = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useSimpleMotor              = false;
        public static final boolean useSimpleServo              = false;
        public static final boolean useElevator                 = false;
        public static final boolean useArm                      = false;
        public static final boolean useShooter                  = false;
        public static final boolean useIntake                   = false;
        public static final boolean useGrabber                  = false;
    }   //class Preferences

    /**
     * This class contains the Robot Hardware Configurations.
     */
    public static class HwConfig
    {
        // Joystick ports.
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int JSPORT_DRIVER_LEFTSTICK         = 0;
        public static final int JSPORT_DRIVER_RIGHTSTICK        = 1;
        public static final int XBOX_OPERATOR_CONTROLLER        = 2;
        public static final int JSPORT_OPERATORSTICK            = 2;
        public static final int JSPORT_BUTTON_PANEL             = 3;
        public static final int JSPORT_SWITCH_PANEL             = 4;
        // CAN IDs.
        // Drive Motor CAN IDs.
        public static final int CANID_LFDRIVE_MOTOR             = 3;
        public static final int CANID_RFDRIVE_MOTOR             = 4;
        public static final int CANID_LBDRIVE_MOTOR             = 5;
        public static final int CANID_RBDRIVE_MOTOR             = 6;
        // Swerve CAN IDs.
        public static final int CANID_LFSTEER_MOTOR             = 13;
        public static final int CANID_RFSTEER_MOTOR             = 14;
        public static final int CANID_LBSTEER_MOTOR             = 15;
        public static final int CANID_RBSTEER_MOTOR             = 16;
        public static final int CANID_LFSTEER_ENCODER           = 23;
        public static final int CANID_RFSTEER_ENCODER           = 24;
        public static final int CANID_LBSTEER_ENCODER           = 25;
        public static final int CANID_RBSTEER_ENCODER           = 26;
        // Miscellaneous CAN IDs.
        public static final int CANID_PCM                       = 30;
        public static final int CANID_PDP                       = 31;
        // Subsystem CAN IDs.

        // PDP Channels.
        // Drive Base PDP Channels.
        public static final ModuleType PDP_MODULE_TYPE          = ModuleType.kRev;
        public static final int PDP_CHANNEL_LFDRIVE_MOTOR       = 11;   // TODO: Need updating
        public static final int PDP_CHANNEL_RFDRIVE_MOTOR       = 5;
        public static final int PDP_CHANNEL_LBDRIVE_MOTOR       = 13;
        public static final int PDP_CHANNEL_RBDRIVE_MOTOR       = 3;
        public static final int PDP_CHANNEL_LFSTEER_MOTOR       = 10;
        public static final int PDP_CHANNEL_RFSTEER_MOTOR       = 6;
        public static final int PDP_CHANNEL_LBSTEER_MOTOR       = 12;
        public static final int PDP_CHANNEL_RBSTEER_MOTOR       = 4;
        // Miscellaneous PDP Channels.
        public static final int PDP_CHANNEL_ROBORIO             = 20;
        public static final int PDP_CHANNEL_VRM                 = 18;
        public static final int PDP_CHANNEL_PCM                 = 19;
        public static final int PDP_CHANNEL_RADIO_POE           = 22;
        public static final int PDP_CHANNEL_ETHERNET_SWITCH     = 21;
        public static final int PDP_CHANNEL_CAMERA              = 0;
        public static final int PDP_CHANNEL_LED                 = 14;

        public static final double BATTERY_CAPACITY_WATT_HOUR   = 18.0*12.0;
        // Analog Input ports (not used).
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;
        public static final int AIN_LFSTEER_ENCODER             = 0;
        public static final int AIN_RFSTEER_ENCODER             = 1;
        public static final int AIN_LBSTEER_ENCODER             = 2;
        public static final int AIN_RBSTEER_ENCODER             = 3;
        // Digital Input/Output ports.

        // PWM channels.
        public static final int NUM_LEDS                        = 60;
        public static final int PWM_CHANNEL_LED                 = 9;
        // Relay channels.

        // Pneumatic channels.

        // Gyro parameters
        public static final String GYRO_NAME                    = "NavX";
        public static final GyroType GYRO_TYPE                  = GyroType.NavX;
        public static final SPI.Port GYRO_PORT                  = SPI.Port.kMXP;

        // Ultrasonic sensors.
        // public static final double SONAR_INCHES_PER_VOLT        = 1.0/0.0098; //9.8mV per inch
        // public static final double SONAR_ERROR_THRESHOLD        = 50.0; //value should not jump 50-in per time slice.
    }   //class HwConfig

    //
    // Robot Parameters.
    //

    /**
     * This class contains the parameters of the front camera.
     */
    public static class FrontCamParams extends FrcRobotDrive.VisionInfo
    {
        public FrontCamParams()
        {
            camName = "OV9281";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = -3.5;                  // Inches to the right from robot center
            camYOffset = -2.375;                // Inches forward from robot center
            camZOffset = 23.125;                // Inches up from the floor
            camPitch = 33.0;                    // degrees up from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //FrontCamParams
    }   //class FrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class BackCamParams extends FrcRobotDrive.VisionInfo
    {
        public BackCamParams()
        {
            camName = "OV9782";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = -0.5;                  // Inches to the right from robot center
            camYOffset = -5.375;                // Inches forward from robot center
            camZOffset = 20.0;                  // Inches up from the floor
            camPitch = -17.5;                   // degrees up from horizontal
            camYaw = 180.0;                     // degrees clockwise from robot front
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //BackCamParams
    }   //class BackCamParams

    /**
     * This class contains the Swerve Robot Parameters.
     */
    public static class SwerveParams extends FrcSwerveDrive.SwerveInfo
    {
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_GEAR_RATIO                    = 6.75;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_GEAR_RATIO                    = 15.43;

        public SwerveParams()
        {
            robotName = "SwerveRobot";
            // Robot Dimensions.
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = 23.25;
            wheelBaseWidth = 23.25;
            // Gyro parameters.
            gyroName = HwConfig.GYRO_NAME;
            gyroType = HwConfig.GYRO_TYPE;
            gyroPort = HwConfig.GYRO_PORT;
            // Drive Motor parameters.
            driveMotorType = MotorType.CanTalonFx;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[] {
                HwConfig.CANID_LFDRIVE_MOTOR, HwConfig.CANID_RFDRIVE_MOTOR,
                HwConfig.CANID_LBDRIVE_MOTOR, HwConfig.CANID_RBDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, false, false, false};
            odometryType = OdometryType.MotorOdometry;
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = yDrivePosScale = DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;    // inch/rev
            // Robot Drive Characteristics
            robotMaxVelocity = 171.0;           // inch/sec
            robotMaxAcceleration = 23000.0;     // inch/sec sq
            robotMaxTurnRate = 1450.0;          // degree/sec
            profiledMaxVelocity = 157.48;       // inch/sec
            profiledMaxAcceleration = 10000.0;  // inch/sec sq
            profiledMaxTurnRate = 180.0;        // degree/sec
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 2.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.017, 0.0, 0.0025, 0.0, 5.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 0.5;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = 0.5;  // %power per sec
            turnPidCoeffs = new PidCoefficients(0.0065, 0.0, 0.0004, 0.0, 10.0);
            turnPidPowerLimit = 1.0;
            turnMaxPidRampRate = 1.0;           // %power per sec
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 10.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / robotMaxVelocity, 0.0);
            // Front Camera
            frontCam = new FrontCamParams();
            // Back Camera
            backCam = new BackCamParams();
            // Steer Encoder parameters.
            steerEncoderType = SteerEncoderType.Canandmag;
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderIds = new int[] {
                HwConfig.CANID_LFSTEER_ENCODER, HwConfig.CANID_RFSTEER_ENCODER,
                HwConfig.CANID_LBSTEER_ENCODER, HwConfig.CANID_RBSTEER_ENCODER};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.0, 0.0, 0.0, 0.0};
            steerZerosFilePath = Robot.STEER_ZERO_CAL_FILE;
            // Steer Motor parameters.
            steerMotorType = MotorType.CanTalonFx;
            steerMotorNames = new String[] {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
            steerMotorIds = new int[] {
                HwConfig.CANID_LFSTEER_MOTOR, HwConfig.CANID_RFSTEER_MOTOR,
                HwConfig.CANID_LBSTEER_MOTOR, HwConfig.CANID_RBSTEER_MOTOR};
            steerMotorInverted = new boolean[] {false, false, false, false};
            steerMotorPidCoeffs = new PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 0.5; // in degrees
            // Swerve Module parameters.
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
            driveGearRatio = DRIVE_GEAR_RATIO;
            //
            // WPILib Parameters.
            //
            invertGyro = true;   // Always ensure Gyro is CCW+ CW-
            // Drivetrain Constants
            wheelBase = Units.inchesToMeters(wheelBaseLength);
            trackWidth = Units.inchesToMeters(wheelBaseWidth);
            wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER * Math.PI);
            // Swerve Kinematics
            // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
            swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
            // Meters per Second
            maxSpeed = Units.inchesToMeters(profiledMaxVelocity);
            // Radians per Second
            maxAngularVelocity = Units.degreesToRadians(profiledMaxTurnRate);
            driveKs = 0.32;
            driveKv = 1.51;
            driveKa = 0.27;

            // double steerScaleDegPerRot = 360.0 / STEER_GEAR_RATIO;
            // // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
            // double steerMaxVelocity = (FALCON_MAX_RPM * 0.81 / STEER_GEAR_RATIO /60.0) * 360.0;
            // // kF set to Motion Magic recommendation.
        }   //SwerveParams
    }   //class SwerveParams

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumParams extends FrcRobotDrive.RobotInfo
    {
        public MecanumParams()
        {
            robotName = "MecanumRobot";
            // Robot Dimensions.
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = 23.25;
            wheelBaseWidth = 23.25;
            // Gyro parameters.
            gyroName = HwConfig.GYRO_NAME;
            gyroType = HwConfig.GYRO_TYPE;
            gyroPort = HwConfig.GYRO_PORT;
            // Drive Motor parameters.
            driveMotorType = MotorType.CanTalonFx;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[]
                {HwConfig.CANID_LFDRIVE_MOTOR, HwConfig.CANID_RFDRIVE_MOTOR,
                 HwConfig.CANID_LBDRIVE_MOTOR, HwConfig.CANID_RBDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, true, false, true};
            odometryType = OdometryType.MotorOdometry;
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = 1.6577438;         // inch/rev
            yDrivePosScale = 2.355935875;       // inch/rev
            // Robot Drive Characteristics
            robotMaxVelocity = 177.1654;        // inch/sec
            robotMaxAcceleration = 799.1;       // inch/sec sq
            robotMaxTurnRate = 572.9578;        // degree/sec
            profiledMaxVelocity = 157.48;       // inch/sec
            profiledMaxAcceleration = 600.0;    // inch/sec sq
            profiledMaxTurnRate = 180.0;        // degree/sec
            // DriveBase PID Parameters
            drivePidTolerance = 2.0;
            turnPidTolerance = 2.0;
            xDrivePidCoeffs = new PidCoefficients(0.017, 0.0, 0.0, 0.0, 0.0);
            xDrivePidPowerLimit = 1.0;
            yDrivePidCoeffs = new PidCoefficients(0.011, 0.0, 0.001, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            turnPidCoeffs = new PidCoefficients(0.012, 0.0, 0.0008, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 12.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / robotMaxVelocity, 0.0);
        }   //MecanumParams
    }   //class MecanumParams

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialParams extends FrcRobotDrive.RobotInfo
    {
        public DifferentialParams()
        {
            robotName = "DifferentialRobot";
            // Robot Dimensions.
            robotLength = Robot.ROBOT_LENGTH;
            robotWidth = Robot.ROBOT_WIDTH;
            wheelBaseLength = 23.25;
            wheelBaseWidth = 23.25;
            // Gyro parameters.
            gyroName = HwConfig.GYRO_NAME;
            gyroType = HwConfig.GYRO_TYPE;
            gyroPort = HwConfig.GYRO_PORT;
            // Drive Motor parameters.
            driveMotorType = MotorType.CanTalonFx;
            driveMotorNames = new String[] {"leftDriveMotor", "rightDriveMotor"};
            driveMotorIds = new int[] {HwConfig.CANID_LFDRIVE_MOTOR, HwConfig.CANID_RFDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, true};
            odometryType = OdometryType.MotorOdometry;
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            yDrivePosScale = 2.355935875;       // inch/rev
            // Robot Drive Characteristics
            robotMaxVelocity = 177.1654;        // inch/sec
            robotMaxAcceleration = 799.1;       // inch/sec sq
            robotMaxTurnRate = 572.9578;        // degree/sec
            profiledMaxVelocity = 157.48;       // inch/sec
            profiledMaxAcceleration = 600.0;    // inch/sec sq
            profiledMaxTurnRate = 180.0;        // degree/sec
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 2.0;
            yDrivePidCoeffs = new PidCoefficients(0.011, 0.0, 0.0013, 0.0, 0.0);
            yDrivePidPowerLimit = 1.0;
            turnPidCoeffs = new PidCoefficients(0.012, 0.0, 0.00008, 0.0, 0.0);
            turnPidPowerLimit = 0.5;
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PurePursuit Parameters
            ppdFollowingDistance = 12.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / robotMaxVelocity, 0.0);
        }   //DifferentialParams
    }   //class DifferentialParams

    public static class VisionOnlyParams extends FrcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            // Front Camera
            frontCam = new FrontCamParams();
            // Back Camera
            backCam = new BackCamParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    public static final class AutoConstants
    {
        //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.0;//0.00149
        public static final double kPYController = 0.0;
        public static final double kPThetaController = 0.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }   //class AutoConstants

    public static class Vision
    {
        public static final double ONTARGET_THRESHOLD           = 5.0;
        public static final double GUIDANCE_ERROR_THRESHOLD     = 12.0;
    }   //class Vision

    /**
     * This class contains Drive parameters of the Drive Base.
     */
    public static class DriveParams
    {
    }   //class DriveParams

    //
    // Other subsystems.
    //

    public static final class Elevator
    {
        public static final String SUBSYSTEM_NAME               = "Elevator";

        public static final String MOTOR_NAME                   = "elevatorMotor";
        public static final int MOTOR_ID                        = 10;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = true;
        public static final double INCHES_PER_COUNT             = 18.25/4941.0;
        public static final double POS_OFFSET                   = 10.875;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 30.25;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 20.0, 25.0, 30.0};
        public static final double POS_PRESET_TOLERANCE         = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.1;
        public static final double GRAVITY_COMP_POWER           = 0.0;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Elevator

    public static final class Arm
    {
        public static final String SUBSYSTEM_NAME               = "Arm";

        public static final String MOTOR_NAME                   = "armMotor";
        public static final int MOTOR_ID                        = 10;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = true;
        public static final double GOBILDA312_CPR               = (((1.0 + (46.0/17.0))) * (1.0 + (46.0/11.0))) * 28.0;
        public static final double DEG_PER_COUNT                = 360.0 / GOBILDA312_CPR;
        public static final double POS_OFFSET                   = 39.0;
        public static final double POWER_LIMIT                  = 0.5;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 270.0;
        public static final double[] posPresets                 = {MIN_POS, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.1, 0.001, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.158;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Arm

    public static final class Shooter
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";

        public static final String MOTOR1_NAME                  = "shooterMotor1";
        public static final int MOTOR1_ID                       = 10;
        public static final MotorType MOTOR1_TYPE               = MotorType.CanTalonSrx;
        public static final boolean MOTOR1_BRUSHLESS            = false;
        public static final boolean MOTOR1_ENC_ABS              = false;
        public static final boolean MOTOR1_INVERTED             = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = true;
        public static final String MOTOR2_NAME                  = "shooterMotor2";
        public static final int MOTOR2_ID                       = 12;
        public static final MotorType MOTOR2_TYPE               = MotorType.CanTalonSrx;
        public static final boolean MOTOR2_BRUSHLESS            = false;
        public static final boolean MOTOR2_ENC_ABS              = false;
        public static final boolean MOTOR2_INVERTED             = true;

        public static final double GOBILDA1620_RPC              = 1.0 / ((1.0 + (46.0/17.0)) * 28.0);
        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients shooter1PidCoeffs =
            new TrcPidController.PidCoefficients(0.025, 0.0, 0.0, 0.039, 0.0);
        public static final TrcPidController.PidCoefficients shooter2PidCoeffs =
            new TrcPidController.PidCoefficients(0.025, 0.0, 0.0, 0.041, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 10.0;

        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 1620.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 100.0;    // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 10.0;     // in RPM
    }   //class Shooter

    public static final class Intake
    {
        public static final String SUBSYSTEM_NAME               = "Intake";
        public static final boolean TWO_MOTOR_INTAKE            = false;

        public static final String PRIMARY_MOTOR_NAME           = "intakePrimaryMotor";
        public static final int PRIMARY_MOTOR_ID                = 10;
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean PRIMARY_MOTOR_BRUSHLESS     = false;
        public static final boolean PRIMARY_MOTOR_ENC_ABS       = false;
        public static final boolean PRIMARY_MOTOR_INVERTED      = !TWO_MOTOR_INTAKE;

        public static final String FOLLOWER_MOTOR_NAME          = "intakeFollowerMotor";
        public static final int FOLLOWER_MOTOR_ID               = 12;
        public static final MotorType FOLLOWER_MOTOR_TYPE       = MotorType.CanTalonSrx;
        public static final boolean FOLLOWER_MOTOR_BRUSHLESS    = false;
        public static final boolean FOLLOWER_MOTOR_ENC_ABS      = false;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = PRIMARY_MOTOR_INVERTED;

        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean SENSOR_INVERTED             = false;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class Intake

    public static final class Grabber
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_SERVO_NAME           = "leftClaw";
        public static final int PRIMARY_SERVO_CHANNEL           = 0;
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = "rightClaw";
        public static final int FOLLOWER_SERVO_CHANNEL          = 1;
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;

        public static final boolean USE_REV_2M_SENSOR           = true;
        public static final double SENSOR_TRIGGER_THRESHOLD     = 2.0;
        public static final double HAS_OBJECT_THRESHOLD         = 2.0;
        public static final boolean ANALOG_TRIGGER_INVERTED     = true;

        public static final boolean USE_DIGITAL_SENSOR          = false;
        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;

    }   //class Grabber

}   //class RobotParams
