/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.studica.frc.AHRS.NavXComType;

import frclib.drivebase.FrcDifferentialBase;
import frclib.drivebase.FrcMecanumBase;
import frclib.drivebase.FrcRobotBase;
import frclib.drivebase.FrcRobotBase.LEDInfo;
import frclib.drivebase.FrcSwerveBase;
import frclib.drivebase.FrcSwerveBase.SteerEncoderMode;
import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcCANCoder;
import frclib.sensor.FrcEncoder.EncoderType;
import teamcode.RobotParams;
import teamcode.RobotParams.HwConfig;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.drivebase.TrcSwerveDrive;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcEncoder;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class DriveBase extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "DriveBase";
    private static final boolean NEED_ZERO_CAL = true;

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // Generic Swerve Drive Base Robot
        SwerveRobot,
        // Generic Mecanum Drive Base Robot
        MecanumRobot,
        // Generic Differential Drive Base Robot
        DifferentialRobot,
        // This is useful for developing Vision code where all you need is a Robot Controller and camera.
        VisionOnly
    }   //enum RobotType

    /**
     * This class contains the Swerve Robot Parameters.
     */
    public static class SwerveRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public final static double DRIVE_WHEEL_DIAMETER         = 3.885547663;  // inches
        public final static double DRIVE_MOTOR_GEAR_RATIO       = 5.51;
        public final static double STEER_MOTOR_GEAR_RATIO       = 468.0/35.1;
        public final static double ROBOT_WIDTH                  = RobotParams.Robot.ROBOT_WIDTH;
        public final static double ROBOT_LENGTH                 = RobotParams.Robot.ROBOT_LENGTH;
        public final static double WHEEL_BASE_WIDTH             = 22.249;
        public final static double WHEEL_BASE_LENGTH            = 22.249;

        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.18, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.FFCoefficients driveMotorVelFFCoeffs =
            new TrcPidController.FFCoefficients(0.25, 0.11, 0.0);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0005, 0.0, 0.0, 0.09, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(52.87825, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.FFCoefficients steerFFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.82872, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                driveMotorVelPidCoeffs, driveMotorVelFFCoeffs, DRIVE_WHEEL_DIAMETER*Math.PI/DRIVE_MOTOR_GEAR_RATIO,
                false)
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(175.0, 210.0, 105.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setFFCoefficients(steerFFCoeffs)
                    .setPidControlParams(0.5, false));

        public SwerveRobotInfo()
        {
            this.setBaseParams(baseParams)
                // maxVel:184.4, maxAcc: 6845.1, maxDecel: 14318.0, turnVel: 423.4
                .setRobotInfo(
                    RobotType.SwerveRobot.toString(), ROBOT_WIDTH, ROBOT_LENGTH,
                    WHEEL_BASE_WIDTH, WHEEL_BASE_LENGTH, 175.0, 400.0)
                .setPigeon2ImuInfo("Pigeon2", RobotParams.HwConfig.CANID_PIGEON2, RobotParams.HwConfig.CANBUS_CANIVORE)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, RobotParams.HwConfig.CANBUS_CANIVORE, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {true, false, true, false})
                .setDriveMotorPosScale(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setWpiOdometry()
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(Vision.cam1Info, Vision.cam2Info)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.CANCoder, RobotParams.HwConfig.CANBUS_CANIVORE,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false}, 1.0,
                    //new double[] {0.125977, 0.687500 , 0.880859 , 0.234863},
                    new double[] {0.126221, 0.35791015625, -0.456787, 0.233398},
                    //new double[] {0.000000, -0.353516, -0.000732, 0.001709},
                    SteerEncoderMode.CtreFusedCanCoder,
                    RobotParams.Robot.teamFolderPath + RobotParams.Robot.STEER_ZERO_CAL_FILE_NAME)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, RobotParams.HwConfig.CANBUS_CANIVORE, null,
                    new String[] {"flSteerMotor", "frSteerMotor", "blSteerMotor", "brSteerMotor"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {true, true, true, true})
                .setSteerPosScale(STEER_MOTOR_GEAR_RATIO, 360.0)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //SwerveRobotInfo
    }   //class SwerveRobotInfo

    /**
     * This class contains the Mecanum Robot Parameters.
     */
    public static class MecanumRobotInfo extends FrcRobotBase.RobotInfo
    {
        private static final TrcPidController.PidCoefficients xDrivePidCoeffs =
            new TrcPidController.PidCoefficients(0.017, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients yDrivePidCoeffs =
            new TrcPidController.PidCoefficients(0.011, 0.0, 0.001, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.012, 0.0, 0.0008, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(xDrivePidCoeffs, 1.0)
            .setYPidParams(yDrivePidCoeffs, 1.0)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(157.48, 600.0, 600.0);

        public MecanumRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.MecanumRobot.toString(), RobotParams.Robot.ROBOT_WIDTH, RobotParams.Robot.ROBOT_LENGTH,
                    23.2, 23.2, 157.48, 180.0)
                .setNavXImuInfo("NavX", NavXComType.kMXP_SPI)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {false, true, false, true})
                .setMotorOdometry(1.6577438, 2.355935875)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(Vision.cam1Info, Vision.cam2Info)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
        }   //MecanumRobotInfo
    }   //class MecanumRobotInfo

    /**
     * This class contains the Differential Robot Parameters.
     */
    public static class DifferentialRobotInfo extends FrcRobotBase.RobotInfo
    {
        private static final TrcPidController.PidCoefficients yDrivePidCoeffs =
            new TrcPidController.PidCoefficients(0.011, 0.0, 0.0013, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.011, 0.0, 0.0013, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.0125, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(2.0, 2.0)
            .setYPidParams(yDrivePidCoeffs, 1.0)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(157.48, 600.0, 600.0);

        public DifferentialRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.DifferentialRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
                    23.2, 23.2, 157.48, 180.0)
                .setNavXImuInfo("NavX", NavXComType.kMXP_SPI)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"leftDriveMotor", "rightDriveMotor"},
                    new int[] {HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR},
                    new boolean[] {false, true})
                .setMotorOdometry(2.355935875)
                .setPidRampRates(0.5, 1.0)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false);
        }   //DifferentialRobotInfo
    }   //class DifferentialRobotInfo

    /**
     * This class contains the VisionOnly Parameters. This is for tuning vision with only the Control Hub and no
     * robot.
     */
    public static class VisionOnlyInfo extends FrcRobotBase.RobotInfo
    {
        public VisionOnlyInfo()
        {
            this.setRobotInfo(RobotType.VisionOnly.toString())
                .setVisionInfo(Vision.hd3000CamInfo, null);
        }   //VisionOnlyInfo
    }   //class VisionOnlyInfo

    private final FrcDashboard dashboard;
    private final FrcRobotBase.RobotInfo robotInfo;
    private final FrcRobotBase robotBase;

    /**
     * Constructor: Create an instance of the object.
     */
    public DriveBase()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        switch (RobotParams.Preferences.robotType)
        {
            case SwerveRobot:
                robotInfo = new SwerveRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcSwerveBase((SwerveRobotInfo) robotInfo): null;
                break;

            case MecanumRobot:
                robotInfo = new MecanumRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcMecanumBase(robotInfo): null;
                break;

            case DifferentialRobot:
                robotInfo = new DifferentialRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcDifferentialBase(robotInfo): null;
                break;

            case VisionOnly:
                robotInfo = new VisionOnlyInfo();
                robotBase = null;
                break;

            default:
                robotInfo = null;
                robotBase = null;
                break;
        }
        configureRobotDrive();
    }   //RobotBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FrcRobotBase.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FrcRobotBase getRobotBase()
    {
        return robotBase;
    }   //getRobotBase

    /**
     * This method configures robotDrive with implementation details.
     */
    private void configureRobotDrive()
    {
        if (robotBase != null)
        {
            if (robotBase instanceof FrcSwerveBase)
            {
                FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
                FrcSwerveBase.SwerveInfo swerveInfo = (FrcSwerveBase.SwerveInfo) robotInfo;
                // Prevent Krakens from browning out.
                for (int i = 0; i < swerveInfo.driveMotorNames.length; i++)
                {
                    swerveBase.driveMotors[i].setCloseLoopRampRate(robotInfo.driveCloseLoopRampRate);
                    swerveBase.driveMotors[i].setCurrentLimit(
                        robotInfo.driveMotorCurrentLimit, robotInfo.driveMotorCurrentTriggerThreshold,
                        robotInfo.driveMotorCurrentTriggerPeriod);
                    swerveBase.driveMotors[i].setStatorCurrentLimit(robotInfo.driveMotorStatorCurrentLimit);
                    if (swerveBase.driveMotors[i] instanceof FrcCANTalonFX)
                    {
                        FrcCANTalonFX driveMotor = (FrcCANTalonFX) swerveBase.driveMotors[i];
                        driveMotor.setFOCEnabled(true);
                    }
                }

                for (int i = 0; i < swerveInfo.steerEncoderNames.length; i++)
                {
                    if (swerveInfo.steerEncoderMode == SteerEncoderMode.SyncToMotorEncoder)
                    {
                        // Sync absolute encoders to steer motor internal encoders.
                        syncSteerEncoder((FrcSwerveBase.SwerveInfo) robotInfo, i);
                    }
                    else if ((swerveInfo.steerEncoderMode == SteerEncoderMode.CtreFusedCanCoder ||
                              swerveInfo.steerEncoderMode == SteerEncoderMode.CtreSyncCanCoder) &&
                             swerveBase.steerEncoders[i] instanceof FrcCANCoder &&
                             swerveBase.steerMotors[i] instanceof FrcCANTalonFX)
                    {
                        FrcCANCoder cancoder = (FrcCANCoder) swerveBase.steerEncoders[i];
                        FrcCANTalonFX steerMotor = (FrcCANTalonFX) swerveBase.steerMotors[i];

                        cancoder.setAbsoluteRange(false);
                        cancoder.setZeroOffset(swerveInfo.steerEncoderZeros[i]);
                        steerMotor.setFOCEnabled(true);
                        steerMotor.setFeedbackDevice(
                            swerveInfo.steerEncoderMode == SteerEncoderMode.CtreFusedCanCoder?
                                FeedbackSensorSourceValue.FusedCANcoder: FeedbackSensorSourceValue.SyncCANcoder,
                            cancoder.getDeviceID(), swerveInfo.steerGearRatio, 1.0, true);
                        // CTRE expects CCW+ but we are CW+, invert steering to correct it.
                        // swerveBase.swerveModules[i].setSteerInverted(true);
                        // TrcDbgTrace.globalTraceDebug(
                        //     moduleName, "%s: Setting ZeroOffset=%f", swerveInfo.steerEncoderZeros[i]);

                    }
                }
            }
        }
    }   //configureRobotDrive

    /**
     * This method reads the absolute steering encoder and synchronize the steering motor encoder with it.
     *
     * @param swerveInfo specifies the swerve drive parameters.
     * @param index specifies the swerve module index.
     */
    private void syncSteerEncoder(FrcSwerveBase.SwerveInfo swerveInfo, int index)
    {
        // Note this method is implementation specific. If your implementation is not with an absolute encoder that
        // syncs with a TalonFX motor, you need to modify this method accordingly.
        FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
        TrcEncoder steerEncoder = swerveBase.steerEncoders[index];
        FrcCANTalonFX steerMotor = (FrcCANTalonFX)swerveBase.steerMotors[index];
        // getPosition returns a value in the range of 0 to 1.0 of one revolution.
        double motorEncoderPos = steerEncoder.getScaledPosition() * swerveInfo.steerGearRatio;
        StatusCode statusCode = steerMotor.motor.setPosition(motorEncoderPos);

        if (statusCode != StatusCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                SUBSYSTEM_NAME,
                swerveInfo.swerveModuleNames[index] + ": TalonFx.setPosition failed (code=" + statusCode +
                ", pos=" + motorEncoderPos + ").");
        }

        double actualEncoderPos = steerMotor.motor.getPosition().getValueAsDouble();
        if (Math.abs(motorEncoderPos - actualEncoderPos) > 0.1)
        {
            TrcDbgTrace.globalTraceWarn(
                SUBSYSTEM_NAME,
                swerveInfo.swerveModuleNames[index] +
                ": Steer encoder out-of-sync (expected=" + motorEncoderPos + ", actual=" + actualEncoderPos + ")");
        }
    }   //syncSteerEncoder

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        if (robotBase != null)
        {
            robotBase.cancel();
        }
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // DriveBase does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // DriveBase does not support resetState.
    }   //resetState

    private static final String DBKEY_SHOW_STATUS       = SUBSYSTEM_NAME + "/ShowStatus";       //Boolean
    private static final String DBKEY_DEBUG_DRIVEBASE   = SUBSYSTEM_NAME + "/DebugDriveBase";   //Boolean
    private static final String DBKEY_DEBUG_PIDDRIVE    = SUBSYSTEM_NAME + "/DebugPidDrive";    //Boolean

    private static final String DBKEY_ROBOT_POSE        = SUBSYSTEM_NAME + "/RobotPose";        //String
    private static final String DBKEY_ROBOT_VEL         = SUBSYSTEM_NAME + "/RobotVel";         //String
    private static final String DBKEY_DRIVE_ENC         = SUBSYSTEM_NAME + "/DriveEnc";         //String
    private static final String DBKEY_STEER_FRONT       = SUBSYSTEM_NAME + "/SteerFront";       //String
    private static final String DBKEY_STEER_BACK        = SUBSYSTEM_NAME + "/SteerBack";        //String
    private static final String DBKEY_XPID_INFO         = SUBSYSTEM_NAME + "/XPidInfo";         //String
    private static final String DBKEY_YPID_INFO         = SUBSYSTEM_NAME + "/YPidInfo";         //String
    private static final String DBKEY_TURNPID_INFO      = SUBSYSTEM_NAME + "/TurnPidInfo";      //String

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_SHOW_STATUS, false);
        dashboard.refreshKey(DBKEY_DEBUG_DRIVEBASE, false);
        dashboard.refreshKey(DBKEY_DEBUG_PIDDRIVE, false);

        dashboard.refreshKey(DBKEY_ROBOT_POSE, "");
        dashboard.refreshKey(DBKEY_ROBOT_VEL, "");
        dashboard.refreshKey(DBKEY_DRIVE_ENC, "");
        dashboard.refreshKey(DBKEY_STEER_FRONT, "");
        dashboard.refreshKey(DBKEY_STEER_BACK, "");
        dashboard.refreshKey(DBKEY_XPID_INFO, "");
        dashboard.refreshKey(DBKEY_YPID_INFO, "");
        dashboard.refreshKey(DBKEY_TURNPID_INFO, "");
    }   //pubishToDashboard

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (robotBase == null)
        {
            return lineNum;
        }

        if (slowLoop && dashboard.getBoolean(DBKEY_SHOW_STATUS, false))
        {
            dashboard.putString(DBKEY_ROBOT_POSE, robotBase.driveBase.getFieldPosition().toString());
            dashboard.putString(DBKEY_ROBOT_VEL, robotBase.driveBase.getRobotVelocity().toString());
            if (dashboard.getBoolean(DBKEY_DEBUG_DRIVEBASE, false))
            {
                // DriveBase debug info.
                double lfDriveEnc =
                    robotBase.driveMotors[MotorIndex.FrontLeft.value].getPosition();
                double rfDriveEnc =
                    robotBase.driveMotors[MotorIndex.FrontRight.value].getPosition();
                double lbDriveEnc =
                    robotBase.driveMotors.length > 2?
                        robotBase.driveMotors[MotorIndex.BackLeft.value].getPosition(): 0.0;
                double rbDriveEnc =
                    robotBase.driveMotors.length > 2?
                    robotBase.driveMotors[MotorIndex.BackRight.value].getPosition(): 0.0;
                dashboard.putString(
                    DBKEY_DRIVE_ENC,
                    String.format(
                        "lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / robotBase.driveMotors.length));
                if (robotBase instanceof FrcSwerveBase)
                {
                    FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
                    dashboard.putString(
                        DBKEY_STEER_FRONT,
                        String.format(
                            "angle/motorEnc/absEnc: lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                            swerveBase.swerveModules[MotorIndex.FrontLeft.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.FrontLeft.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.FrontLeft.value].getRawPosition(),
                            swerveBase.swerveModules[MotorIndex.FrontRight.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.FrontRight.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.FrontRight.value].getRawPosition()));
                    dashboard.putString(
                        DBKEY_STEER_BACK,
                        String.format(
                            "angle/motorEnc/absEnc: lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                            swerveBase.swerveModules[MotorIndex.BackLeft.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.BackLeft.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.BackLeft.value].getRawPosition(),
                            swerveBase.swerveModules[MotorIndex.BackRight.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.BackRight.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.BackRight.value].getRawPosition()));
                }
            }

            if (dashboard.getBoolean(DBKEY_DEBUG_PIDDRIVE, false))
            {
                TrcPidController pidCtrl = robotBase.pidDrive.getXPidCtrl();
                double[] pidInfo;
                if (pidCtrl != null)
                {
                    pidInfo = pidCtrl.getPidStateInfo();
                    dashboard.putString(
                        DBKEY_XPID_INFO,
                        String.format(
                            "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                            pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                            pidInfo[6]));
                }
                pidCtrl = robotBase.pidDrive.getYPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    DBKEY_YPID_INFO,
                    String.format(
                        "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                        pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                        pidInfo[6]));
                pidCtrl = robotBase.pidDrive.getTurnPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    DBKEY_TURNPID_INFO,
                    String.format(
                        "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                        pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                        pidInfo[6]));
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     * @param nextValueUp specifies true for the next preset target value up, false for next preset target value down,
     *        null for the current target value.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName, Boolean nextValueUp)
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsFromDashboard(String subsystemName)
    {
    }   //updateParamsFromDashboard

}   //class RobotDrive
