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

package team492;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team492.drivebases.RobotDrive.DriveMode;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.vision.TrcHomographyMapper;

/**
 * This class contains parameters and preferences related to all robot operations.
 */
public class RobotParams
{
    public enum RobotType
    {
        NoRobot,
        DifferentialRobot,
        MecanumRobot,
        SwerveRobot
    }   //enum RobotType

    public enum SteerEncoderType
    {
        CANCoder,
        Canandmag,
        AnalogEncoder
    }   //enum SteerEncoderType

    //
    // Robot preferences.
    //
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
        public static final boolean showSubsystems              = false;
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
        public static final boolean useExternalOdometry         = false;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        public static final boolean useSteeringCANCoder         = false;
        public static final boolean useSteeringCanandmag        = true;
        public static final boolean useSteeringAnalogEncoder    = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
    }   //class Preferences

    public static final String TEAM_FOLDER_PATH                 = "/home/lvuser/trc492";
    public static final double DASHBOARD_UPDATE_INTERVAL        = 0.1;      // in msec

    public static class Field
    {
        // Field dimensions in inches.
        public static final double LENGTH                       = 54.0*12.0;
        public static final double WIDTH                        = 27.0*12.0;
    }   //class Field

    public static class Robot
    {
        public static final String NAME                         = "Robot492";
        // Robot dimensions in inches.
        public static final double WIDTH                        = 34.5;     // Frame dimensions, including bumpers.
        public static final double LENGTH                       = 34.5;     // Frame dimensions, including bumpers.

        public static final double WHEELBASE_WIDTH              = 23.25;    // Required by swerve drive base.
        public static final double WHEELBASE_LENGTH             = 23.25;    // Required by swerve drive base.
    }   //class Robot

    public static class Game
    {
        //
        // Robot starting positions.
        //
        public static final double STARTPOS_BLUE_Y              = Robot.LENGTH / 2.0;
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

    public static class HWConfig
    {
        //
        // Joystick ports.
        //
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int JSPORT_DRIVER_LEFTSTICK         = 0;
        public static final int JSPORT_DRIVER_RIGHTSTICK        = 1;
        public static final int XBOX_OPERATOR_CONTROLLER        = 2;
        public static final int JSPORT_OPERATORSTICK            = 2;
        public static final int JSPORT_BUTTON_PANEL             = 3;
        public static final int JSPORT_SWITCH_PANEL             = 4;
        //
        // CAN IDs.
        //
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

        //
        // PDP Channels.
        //
        // Drive Base PDP Channels.
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
        //
        // Analog Input ports (not used).
        //
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;
        public static final int AIN_LFSTEER_ENCODER             = 0;
        public static final int AIN_RFSTEER_ENCODER             = 1;
        public static final int AIN_LBSTEER_ENCODER             = 2;
        public static final int AIN_RBSTEER_ENCODER             = 3;
        //
        // Digital Input/Output ports.
        //

        //
        // PWM channels.
        //
        public static final int NUM_LEDS                        = 60;
        public static final int PWM_CHANNEL_LED                 = 9;
        //
        // Relay channels.
        //

        //
        // Pneumatic channels.
        //

        //
        // Ultrasonic sensors.
        //
        // public static final double SONAR_INCHES_PER_VOLT        = 1.0/0.0098; //9.8mV per inch
        // public static final double SONAR_ERROR_THRESHOLD        = 50.0; //value should not jump 50-in per time slice.
    }   //class HWConfig

    public static class Vision
    {
        public static final int FRONTCAM_IMAGE_WIDTH            = 1280;     // in pixels
        public static final int FRONTCAM_IMAGE_HEIGHT           = 800;      // in pixels
        // Camera location on robot.
        public static final double FRONTCAM_X_OFFSET            = -3.5;     // Inches to the right from robot center
        public static final double FRONTCAM_Y_OFFSET            = -2.375;   // Inches forward from robot center
        public static final double FRONTCAM_Z_OFFSET            = 23.125;   // Inches up from the floor
        public static final double FRONTCAM_PITCH               = 33.0;     // degrees up from horizontal
        public static final double FRONTCAM_YAW                 = 0.0;      // degrees clockwise from robot front
        public static final double FRONTCAM_ROLL                = 0.0;
        public static final Transform3d robotToFrontCam         = new Transform3d(
            new Translation3d(Units.inchesToMeters(FRONTCAM_Y_OFFSET), -Units.inchesToMeters(FRONTCAM_X_OFFSET),
                              Units.inchesToMeters(FRONTCAM_Z_OFFSET)),
            new Rotation3d(Units.degreesToRadians(FRONTCAM_ROLL), Units.degreesToRadians(-FRONTCAM_PITCH),
                           Units.degreesToRadians(-FRONTCAM_YAW)));
        public static final TrcPose2D robotToFrontCamPose       = new TrcPose2D(
            FRONTCAM_X_OFFSET, FRONTCAM_Y_OFFSET, FRONTCAM_YAW);

        public static final int BACKCAM_IMAGE_WIDTH             = 1280;     // in pixels
        public static final int BACKCAM_IMAGE_HEIGHT            = 800;      // in pixels
        // Camera location on robot.
        public static final double BACKCAM_X_OFFSET             = -0.5;     // Inches to the right from robot center
        public static final double BACKCAM_Y_OFFSET             = -5.375;   // Inches forward from robot center
        public static final double BACKCAM_Z_OFFSET             = 20.0;     // Inches up from the floor
        public static final double BACKCAM_PITCH                = -17.5;    // degrees up from horizontal
        public static final double BACKCAM_YAW                  = 180.0;    // degrees clockwise from robot front
        public static final double BACKCAM_ROLL                 = 0.0;
        public static final Transform3d robotToBackCam          = new Transform3d(
            new Translation3d(Units.inchesToMeters(BACKCAM_Y_OFFSET), -Units.inchesToMeters(BACKCAM_X_OFFSET),
                              Units.inchesToMeters(BACKCAM_Z_OFFSET)),
            new Rotation3d(Units.degreesToRadians(BACKCAM_ROLL), Units.degreesToRadians(-BACKCAM_PITCH),
            Units.degreesToRadians(-BACKCAM_YAW)));
        public static final TrcPose2D robotToBackCamPose        = new TrcPose2D(
            BACKCAM_X_OFFSET, BACKCAM_Y_OFFSET, BACKCAM_YAW);
        // Camera: Logitech C310 (not used)
        public static final double WEBCAM_FX                    = 821.993;  // in pixels
        public static final double WEBCAM_FY                    = 821.993;  // in pixels
        public static final double WEBCAM_CX                    = 330.489;  // in pixels
        public static final double WEBCAM_CY                    = 248.997;  // in pixels

        public static final double CAMERA_DATA_TIMEOUT          = 0.5;      // 500ms
        public static final double VISION_TARGET_HEIGHT         = 104.0;    // Inches from the floor (not used)
        public static final double APRILTAG_SIZE                = Units.inchesToMeters(6.5);    //  in meters
        // Homography measurements (not used).
        // Camera rect in inches.
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_X  = 0;
        public static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y  = 400.0;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X = BACKCAM_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y = 400.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X = 0.0;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y = BACKCAM_IMAGE_HEIGHT - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X = BACKCAM_IMAGE_WIDTH - 1;
        public static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y = BACKCAM_IMAGE_HEIGHT - 1;
        public static final TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
            HOMOGRAPHY_CAMERA_TOPLEFT_X, HOMOGRAPHY_CAMERA_TOPLEFT_Y,
            HOMOGRAPHY_CAMERA_TOPRIGHT_X, HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
            HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);
        // World rect in inches.
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_X   = -46.0;
        public static final double HOMOGRAPHY_WORLD_TOPLEFT_Y   =  76.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_X  = 33.0;
        public static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y  = 73.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X= -17.0;
        public static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y= 42.5 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X = 15.0;
        public static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y = 41.0 - Robot.LENGTH/2.0 + BACKCAM_Y_OFFSET;
        public static final TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
            HOMOGRAPHY_WORLD_TOPLEFT_X, HOMOGRAPHY_WORLD_TOPLEFT_Y,
            HOMOGRAPHY_WORLD_TOPRIGHT_X, HOMOGRAPHY_WORLD_TOPRIGHT_Y,
            HOMOGRAPHY_WORLD_BOTTOMLEFT_X, HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
            HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

        public static final double ONTARGET_THRESHOLD           = 5.0;
        public static final double GUIDANCE_ERROR_THRESHOLD     = 12.0;
    }   //class Vision

    //
    // Subsystems.
    //
    // DriveBase Subsystem.
    public static class DriveParams
    {
        public static final DriveMode ROBOT_DRIVE_MODE          = DriveMode.ArcadeMode;
        public static final double DRIVE_RAMP_RATE              = 0.25;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;

        public static double robotMaxVelocity;
        public static double robotMaxAcceleration;
        public static double robotMaxTurnRate;
        public static double profiledMaxVelocity;
        public static double profiledMaxAcceleration;
        public static double profiledMaxTurnRate;
    }   //class DriveParams

    public static class DriveBaseParams
    {
        // Drive Notors
        public String[] driveMotorNames;
        public int[] driveMotorIds;
        public boolean[] driveMotorInverted;
        public double xDriveScaleInchPerRot, yDriveScaleInchPerRot;
        // DriveBase Characteristics
        // DriveBase PID Parameters
        public TrcPidController.PidCoefficients xDrivePidCoeff, yDrivePidCoeff;
        public double drivePidTolerance;
        public double xDriveMaxPidPower, yDriveMaxPidPower;
        public double xDriveMaxPidRampRate, yDriveMaxPidRampRate;
        public TrcPidController.PidCoefficients turnPidCoeff;
        public double turnPidTolerance;
        public double turnMaxPidPower;
        public double turnMaxPidRampRate;
        public TrcPidController.PidCoefficients velPidCoeff;
        public TrcPidController.PidCoefficients xTippingPidCoeff;
        public TrcPidController.PidCoefficients yTippingPidCoeff;
        public double tippingPidTolerance;
    }   //class DriveBaseParams

    public static class SwerveDriveBase extends DriveBaseParams
    {
        public static final String STEER_ZERO_CAL_FILE          = "SteerCalibration.txt";
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_GEAR_RATIO                    = 6.75;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_GEAR_RATIO                    = 15.43;
        public final double PPD_FOLLOWING_DISTANCE              = 10.0;
        public final double PPD_POS_TOLERANCE                   = 2.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 1.0;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;
        // Drive Motor Characterization Values From SYSID
        public final double DRIVE_KS                            = 0.32;     //TODO: This must be tuned to specific robot
        public final double DRIVE_KV                            = 1.51;
        public final double DRIVE_KA                            = 0.27;
        // Steer Motors
        public String[] steerMotorNames;
        public int[] steerMotorIds;
        public boolean[] steerMotorInverted;
        public double steerScaleDegPerRot;
        public double steerMaxVelocity;
        public TrcPidController.PidCoefficients steerPosPidCoeff;
        public double steerPosPidTolerance;
        // Steer Encoders
        public SteerEncoderType steerEncoderType;
        public String[] steerEncoderNames;
        public int[] steerEncoderIds;
        public boolean[] steerEncoderInverted;
        public double[] steerEncoderZeros;
        // Swerve Modules
        public String[] swerveModuleNames;
        //
        // Command-based constants.
        //
        public boolean invertGyro;
        public double trackWidth;
        public double wheelBase;
        public double wheelCircumference;
        public SwerveDriveKinematics swerveKinematics;
        public double maxSpeed;
        public double maxAngularVelocity;

        public SwerveDriveBase()
        {
            // Drive Motors
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[] {
                HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR,
                HWConfig.CANID_LBDRIVE_MOTOR, HWConfig.CANID_RBDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, false, false, false};
            xDriveScaleInchPerRot = yDriveScaleInchPerRot = DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;

            // Steer Motors
            steerMotorNames = new String[] {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
            steerMotorIds = new int[] {
                HWConfig.CANID_LFSTEER_MOTOR, HWConfig.CANID_RFSTEER_MOTOR,
                HWConfig.CANID_LBSTEER_MOTOR, HWConfig.CANID_RBSTEER_MOTOR};
            steerMotorInverted = new boolean[] {false, false, false, false};
            steerScaleDegPerRot = 360.0 / STEER_GEAR_RATIO;
            // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
            steerMaxVelocity = (FALCON_MAX_RPM * 0.81 / STEER_GEAR_RATIO /60.0) * 360.0;
            // kF set to Motion Magic recommendation.
            steerPosPidCoeff = new PidCoefficients(3.0, 0.0, 0.0, 0.0, 5.0);
            steerPosPidTolerance = 0.5; // in degrees

            // Steer Encoders
            steerEncoderType = SteerEncoderType.Canandmag;
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderIds = new int[] {
                HWConfig.CANID_LFSTEER_ENCODER, HWConfig.CANID_RFSTEER_ENCODER,
                HWConfig.CANID_LBSTEER_ENCODER, HWConfig.CANID_RBSTEER_ENCODER};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            // Zeroes are normalized offsets which are in the unit of percentage revolution (0.0 to 1.0).
            // This is a backup if file is not found: LF, RF, LB, RB.
            steerEncoderZeros = new double[] {0.0, 0.0, 0.0, 0.0};

            // Swerve Modules
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};

            // Drive Characteristics
            DriveParams.robotMaxVelocity = 171.0;           // inches per sec
            DriveParams.robotMaxAcceleration = 23000.0;     // inches per sec square
            DriveParams.robotMaxTurnRate = 1450.0;          // degrees per sec
            DriveParams.profiledMaxVelocity = 157.48;       // inches per sec
            DriveParams.profiledMaxAcceleration = 10000.0;  // inches per sec square
            DriveParams.profiledMaxTurnRate = 180.0;        // degrees per sec
            // DriveBase PID Parameters
            xDrivePidCoeff = yDrivePidCoeff = new PidCoefficients(0.017, 0.0, 0.0025, 0.0, 5.0);
            drivePidTolerance = 1.0;
            xDriveMaxPidPower = yDriveMaxPidPower = 0.5;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = 0.5;// %power per sec
            turnPidCoeff = new PidCoefficients(0.0065, 0.0, 0.0004, 0.0, 10.0);
            turnPidTolerance = 2.0;
            turnMaxPidPower = 1.0;
            turnMaxPidRampRate = 1.0;                       // %power per sec
            // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance),
            // units: sec./in.
            velPidCoeff = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / DriveParams.profiledMaxVelocity);
            // Not tuned (not used).
            xTippingPidCoeff = yTippingPidCoeff = new PidCoefficients(0.01, 0.0, 0.0, 10.0);
            tippingPidTolerance = 10.0;
            //
            // Command-based Parameters.
            //
            invertGyro = true;  // Always ensure Gyro is CCW+ CW-
            // Drivetrain Constants
            trackWidth = Units.inchesToMeters(Robot.WHEELBASE_WIDTH);
            wheelBase = Units.inchesToMeters(Robot.WHEELBASE_LENGTH);
            wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER * Math.PI);
            // Swerve Kinematics
            // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
            swerveKinematics  = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
            // Meters per Second
            maxSpeed = Units.inchesToMeters(DriveParams.profiledMaxVelocity);
            // Radians per Second
            maxAngularVelocity = Units.degreesToRadians(DriveParams.profiledMaxTurnRate);
        }
    }   //class SwerveDriveBase

    public static class MecanumDriveBase extends DriveBaseParams
    {
        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;

        public MecanumDriveBase()
        {
            // Drive Motors
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[] {
                HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR,
                HWConfig.CANID_LBDRIVE_MOTOR, HWConfig.CANID_RBDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, true, false, true};
            xDriveScaleInchPerRot = 1.6577438;
            yDriveScaleInchPerRot = 2.355935875;

            // Drive Characteristics
            DriveParams.robotMaxVelocity = 177.1654;        // inches per sec
            DriveParams.robotMaxAcceleration = 799.1;       // inches per sec square
            DriveParams.robotMaxTurnRate = 572.9578;        // degrees per sec
            DriveParams.profiledMaxVelocity = 157.48;       // inches per sec
            DriveParams.profiledMaxAcceleration = 600.0;    // inches per sec square
            DriveParams.profiledMaxTurnRate = 180.0;        // degrees per sec
            // DriveBase PID Parameters
            xDrivePidCoeff = new PidCoefficients(0.017, 0.0, 0.0, 0.0, 0.0);
            yDrivePidCoeff = new PidCoefficients(0.011, 0.001, 0.0, 0.0, 0.0);
            drivePidTolerance = 2.0;
            xDriveMaxPidPower = yDriveMaxPidPower = 0.5;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = 0.5;// %power per sec
            turnPidCoeff = new PidCoefficients(0.012, 0.0, 0.00008, 0.0, 0.0);
            turnPidTolerance = 2.0;
            turnMaxPidPower = 1.0;
            turnMaxPidRampRate = 1.0;                       // %power per sec
            // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance),
            // units: sec./in.
            velPidCoeff = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / DriveParams.profiledMaxVelocity);
            // Not tuned (not used).
            xTippingPidCoeff = new PidCoefficients(0.017, 0.0, 0.0, 10.0);
            yTippingPidCoeff = new PidCoefficients(0.01, 0.0, 0.0, 10.0);
            tippingPidTolerance = 10.0;
        }
    }   //class MecanumDriveBase

    public static class DifferentialDriveBase extends DriveBaseParams
    {
        public final double PPD_FOLLOWING_DISTANCE              = 12.0;
        public final double PPD_POS_TOLERANCE                   = 1.0;
        public final double PPD_TURN_TOLERANCE                  = 2.0;
        public final double PPD_MOVE_DEF_OUTPUT_LIMIT           = 0.5;
        public final double PPD_ROT_DEF_OUTPUT_LIMIT            = 0.5;

        public DifferentialDriveBase()
        {
            // Drive Motors
            driveMotorNames = new String[] {"leftDriveMotor", "rightDriveMotor"};
            driveMotorIds = new int[] {HWConfig.CANID_LFDRIVE_MOTOR, HWConfig.CANID_RFDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, true};
            yDriveScaleInchPerRot = 2.355935875;

            // Drive Characteristics
            DriveParams.robotMaxVelocity = 177.1654;        // inches per sec
            DriveParams.robotMaxAcceleration = 799.1;       // inches per sec square
            DriveParams.robotMaxTurnRate = 572.9578;        // degrees per sec
            DriveParams.profiledMaxVelocity = 157.48;       // inches per sec
            DriveParams.profiledMaxAcceleration = 600.0;    // inches per sec square
            DriveParams.profiledMaxTurnRate = 180.0;        // degrees per sec
            // DriveBase PID Parameters
            yDrivePidCoeff = new PidCoefficients(0.011, 0.001, 0.0, 0.0, 0.0);
            drivePidTolerance = 2.0;
            yDriveMaxPidPower = 0.5;
            yDriveMaxPidRampRate = 0.5;                     // %power per sec
            turnPidCoeff = new PidCoefficients(0.012, 0.0, 0.00008, 0.0, 0.0);
            turnPidTolerance = 2.0;
            turnMaxPidPower = 1.0;
            turnMaxPidRampRate = 1.0;                       // %power per sec
            // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance),
            // units: sec./in.
            velPidCoeff = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / DriveParams.profiledMaxVelocity);
            // Not tuned (not used).
            yTippingPidCoeff = new PidCoefficients(0.01, 0.0, 0.0, 10.0);
            tippingPidTolerance = 10.0;
        }
    }   //class DifferentialDriveBase

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

    //
    // Other subsystems.
    //

}   //class RobotParams
