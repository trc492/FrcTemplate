/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcServoActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcShooter;
import teamcode.FrcTest;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcDiscreteValue;
import trclib.dataprocessor.TrcLookupTable;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Shooter Subsystem. This implementation consists of one or two shooter motors. For
 * two-motor shooter, the two motors can be arranged to spin in the same direction (2-stage shooting) or in opposite
 * directions. For opposite spinning motor arrangement, one can spin the motors at different speed to create back spin
 * when shooting the object. In the two-motor configuration, because the two motors may not be identical (even if they
 * are the same model), the subsystem allows you to tune different PID coefficients for each motor. The shooter
 * subsystem also supports optionally mounting on a pan and tilt platform. This allows for aiming the shooter at
 * the shooting target.
 */
public class Shooter extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = false;
    private static final double GOBILDA6000_CPR = 28.0;

    private static final boolean HAS_TWO_SHOOTER_MOTORS = false;
    private static final boolean HAS_PAN_MOTOR = false;
    private static final boolean HAS_TILT_MOTOR = false;
    private static final boolean HAS_LAUNCHER = false;

    public static class ShooterMotorParams
    {
        // Shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;

        public static final String MOTOR1_NAME                  = SUBSYSTEM_NAME + ".shooterMotor1";
        public static final int MOTOR1_ID                       = 10;
        public static final boolean MOTOR1_INVERTED             = false;
        public static final boolean MOTOR1_VOLTCOMP_ENABLED     = true;
        public static final boolean MOTOR1_BRAKE_ENABLED        = false;

        public static final TrcPidController.PidCoefficients motor1PidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0085, 0.0);
        public static final TrcPidController.FFCoefficients motor1FFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.0, 0.0);

        public static final String MOTOR2_NAME                  = SUBSYSTEM_NAME + ".shooterMotor2";
        public static final int MOTOR2_ID                       = 12;
        public static final boolean MOTOR2_INVERTED             = true;
        public static final boolean MOTOR2_VOLTCOMP_ENABLED     = true;
        public static final boolean MOTOR2_BRAKE_ENABLED        = false;

        public static final TrcPidController.PidCoefficients motor2PidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0085, 0.0);
        public static final TrcPidController.FFCoefficients motor2FFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.008, 0.0);

        public static final double PID_TOLERANCE                = 1.0;      // in RPS (60 RPM)
        public static final boolean USE_SOFTWARE_PID            = true;

        public static final double GEAR_RATIO                   = 24.0/36.0;
        public static final double REV_PER_COUNT                = 1.0/(GOBILDA6000_CPR * GEAR_RATIO);

        public static final double OFF_DELAY                    = 0.5;      // in sec

        // These are for tuning shooter motor with gamepad.
        public static final double MIN_VEL                      = 10.0;     // in RPM
        public static final double MAX_VEL                      = 7360.0;   // in RPM
        public static final double MIN_VEL_INC                  = 1.0;      // in RPM
        public static final double MAX_VEL_INC                  = 1000.0;   // in RPM
        public static final double DEF_VEL                      = 1000.0;   // in RPM
        public static final double DEF_VEL_INC                  = 100.0;    // in RPM
    }   //class ShooterMotorParams

    public static class PanMotorParams
    {
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".panMotor";
        public static final int MOTOR_ID                        = 14;
        public static final boolean MOTOR_INVERTED              = false;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final double PID_TOLERANCE                = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients pidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);

        public static final double DEG_PER_COUNT                = 1.0;
        public static final double POS_OFFSET                   = -90.0;
        public static final double MIN_POS                      = -90.0;
        public static final double FRONT_POS                    = 0.0;
        public static final double MAX_POS                      = 90.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, -60.0, -30.0, 0.0, 30.0, 60.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.2;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class PanMotorParams

    public static class TiltMotorParams
    {
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".tiltMotor";
        public static final int MOTOR_ID                        = 16;
        public static final boolean MOTOR_INVERTED              = false;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final double PID_TOLERANCE                = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients pidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);

        public static final double DEG_PER_COUNT                = 1.0;
        public static final double POS_OFFSET                   = 0.0;
        public static final double MIN_POS                      = 0.0;
        public static final double MAX_POS                      = 90.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 30.0, 45.0, 60.0, 75.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
    }   //class TiltMotorParams

    public static class LauncherParams
    {
        public static final String SERVO_NAME                   = SUBSYSTEM_NAME + ".launcher";
        public static final int SERVO_CHANNEL                   = 0;
        public static final boolean SERVO_INVERTED              = true;
        public static double REST_POS                           = 0.0;
        public static double LAUNCH_POS                         = 0.5;
        public static double LAUNCH_DURATION                    = 0.5;  // in seconds
    }   //class LauncherParams

    public static final TrcPose2D robotToShooterPose = new TrcPose2D(0.0, 0.0, 0.0);

    private static final TrcLookupTable.Region[] regions =
    {
        new TrcLookupTable.Region(60.0, new double[][] {null})
    };

    public static final TrcLookupTable shootParamTable = new TrcLookupTable()
        //        name,                 distance,   region,             ShooterVel(rps)
        .addEntry(null,                 36.0,       regions[0],         60.0)
        .addEntry(null,                 48.0,       regions[0],         70.0)
        .addEntry(null,                 60.0,       regions[0],         80.0)
        .addEntry(null,                 72.0,       regions[0],         90.0);

    private final FrcDashboard dashboard;
    private final TrcShooter shooter;
    public final TrcDiscreteValue shooter1Velocity;
    public final TrcDiscreteValue shooter2Velocity;
    public final TrcServo launcher;

    private String launchOwner;
    private TrcEvent launchCompletionEvent;
    private TrcEvent launchCallbackEvent = null;

    private String tuneSubsystemName = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                ShooterMotorParams.MOTOR1_NAME, ShooterMotorParams.MOTOR_TYPE, ShooterMotorParams.MOTOR1_INVERTED,
                ShooterMotorParams.MOTOR1_VOLTCOMP_ENABLED, ShooterMotorParams.MOTOR1_BRAKE_ENABLED,
                ShooterMotorParams.MOTOR1_ID, null, null, false);

        if (HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                ShooterMotorParams.MOTOR2_NAME, ShooterMotorParams.MOTOR_TYPE, ShooterMotorParams.MOTOR2_INVERTED,
                ShooterMotorParams.MOTOR2_VOLTCOMP_ENABLED, ShooterMotorParams.MOTOR2_BRAKE_ENABLED,
                ShooterMotorParams.MOTOR2_ID, null, null, false, true);
        }

        if (HAS_PAN_MOTOR)
        {
            shooterParams.setPanMotor(
                PanMotorParams.MOTOR_NAME, PanMotorParams.MOTOR_TYPE, PanMotorParams.MOTOR_INVERTED,
                PanMotorParams.MOTOR_VOLTCOMP_ENABLED, PanMotorParams.MOTOR_BRAKE_ENABLED,
                PanMotorParams.MOTOR_ID, null, null,
                new TrcShooter.PanTiltParams(
                    PanMotorParams.POWER_LIMIT, PanMotorParams.MIN_POS, PanMotorParams.MAX_POS));
            shooterParams.setPanMotorPosPresets(PanMotorParams.POS_PRESET_TOLERANCE, PanMotorParams.posPresets);
        }

        if (HAS_TILT_MOTOR)
        {
            shooterParams.setTiltMotor(
                TiltMotorParams.MOTOR_NAME, TiltMotorParams.MOTOR_TYPE, TiltMotorParams.MOTOR_INVERTED,
                TiltMotorParams.MOTOR_VOLTCOMP_ENABLED, TiltMotorParams.MOTOR_BRAKE_ENABLED,
                TiltMotorParams.MOTOR_ID, null, null, 
                new TrcShooter.PanTiltParams(
                    TiltMotorParams.POWER_LIMIT, TiltMotorParams.MIN_POS, TiltMotorParams.MAX_POS));
            shooterParams.setTiltMotorPosPresets(TiltMotorParams.POS_PRESET_TOLERANCE, TiltMotorParams.posPresets);
        }

        shooter = new FrcShooter(SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(ShooterMotorParams.REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(ShooterMotorParams.motor1PidCoeffs)
                .setFFCoefficients(ShooterMotorParams.motor1FFCoeffs)
                .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID), null);
        // For tuning shooter motor 1 PID.
        shooter1Velocity = new TrcDiscreteValue(
            SUBSYSTEM_NAME + ".motor1TargetVel",
            ShooterMotorParams.MIN_VEL, ShooterMotorParams.MAX_VEL,
            ShooterMotorParams.MIN_VEL_INC, ShooterMotorParams.MAX_VEL_INC,
            ShooterMotorParams.DEF_VEL, ShooterMotorParams.DEF_VEL_INC);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(ShooterMotorParams.REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(ShooterMotorParams.motor2PidCoeffs)
                    .setFFCoefficients(ShooterMotorParams.motor2FFCoeffs)
                    .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID), null);
            // For tuning shooter motor 2 PID.
            shooter2Velocity = new TrcDiscreteValue(
                SUBSYSTEM_NAME + ".motor2TargetVel",
                ShooterMotorParams.MIN_VEL, ShooterMotorParams.MAX_VEL,
                ShooterMotorParams.MIN_VEL_INC, ShooterMotorParams.MAX_VEL_INC,
                ShooterMotorParams.DEF_VEL, ShooterMotorParams.DEF_VEL_INC);
        }
        else
        {
            shooter2Velocity = null;
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(PanMotorParams.DEG_PER_COUNT, PanMotorParams.POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(PanMotorParams.pidCoeffs)
                    .setPidControlParams(PanMotorParams.PID_TOLERANCE, PanMotorParams.USE_SOFTWARE_PID),
                null);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                PanMotorParams.STALL_MIN_POWER, PanMotorParams.STALL_TOLERANCE, PanMotorParams.STALL_TIMEOUT,
                PanMotorParams.STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(PanMotorParams.MIN_POS, PanMotorParams.MAX_POS, false);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(TiltMotorParams.DEG_PER_COUNT, TiltMotorParams.POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(TiltMotorParams.pidCoeffs)
                    .setPidControlParams(TiltMotorParams.PID_TOLERANCE, TiltMotorParams.USE_SOFTWARE_PID),
                null);
            motor.setSoftPositionLimits(TiltMotorParams.MIN_POS, TiltMotorParams.MAX_POS, false);
        }

        if (HAS_LAUNCHER)
        {
            FrcServoActuator.Params launcherParams = new FrcServoActuator.Params()
                .setPrimaryServo(
                    LauncherParams.SERVO_NAME, LauncherParams.SERVO_CHANNEL, LauncherParams.SERVO_INVERTED);
            launcher = new FrcServoActuator(launcherParams).getServo();
        }
        else
        {
            launcher = null;
        }
    }   //Shooter

    /**
     * This method returns the created shooter.
     *
     * @return created shooter.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    /**
     * This method is called to launch the game piece into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     * @param context specifies the context object passed to the ShootOp method.
     */
    public void shoot(String owner, TrcEvent completionEvent, Object context)
    {
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        if (launcher != null)
        {
            TrcDbgTrace.globalTraceInfo(instanceName, "shoot(owner=" + owner + ", event=" + completionEvent + ")");
            launchOwner = owner;
            launchCompletionEvent = completionEvent;
            launchCallbackEvent = new TrcEvent(SUBSYSTEM_NAME + ".launchCallback");
            launchCallbackEvent.setCallback(this::launchCallback, null);
            launcher.setPosition(
                owner, 0.0, LauncherParams.LAUNCH_POS, launchCallbackEvent, LauncherParams.LAUNCH_DURATION);
        }
        else if (completionEvent != null)
        {
            TrcDbgTrace.globalTraceInfo(instanceName, "There is no launcher, signal completion anyway.");
            completionEvent.signal();
        }
    }   //shoot

    /**
     * This method is called when the launch duration has expired.
     *
     * @param context not used.
     * @param canceled specifies true if launch was canceled (not used).
     */
    private void launchCallback(Object context, boolean canceled)
    {
        // Reset launcher, fire and forget.
        launcher.setPosition(launchOwner, 0.0, LauncherParams.REST_POS, null, 0.0);
        if (launchCompletionEvent != null)
        {
            if (canceled)
            {
                launchCompletionEvent.cancel();
            }
            else
            {
                launchCompletionEvent.signal();
            }
            launchCompletionEvent = null;
        }
        launchOwner = null;
        launchCallbackEvent = null;
    }   //launchCallback

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        shooter.cancel();
        if (launcher != null)
        {
            launcher.cancel();
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
        // Shooter does not need zero calibration.
        // Tilter has absolute encoder and therefore no need for zero calibration.
        // Zero calibrate turret (pan).
        shooter.panMotor.zeroCalibrate(owner, PanMotorParams.ZERO_CAL_POWER, completionEvent, 0.0);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Shooter does not support resetState.
        // If you need to tuck away pan and tilt for turtle mode, add code here.
    }   //resetState

    private static final String DBKEY_PWR1_INFO         = SUBSYSTEM_NAME + "/ShooterPwr1Info";      //String
    private static final String DBKEY_VEL1_INFO         = SUBSYSTEM_NAME + "/ShooterVel1Info";      //String
    private static final String DBKEY_VEL1_RPM          = SUBSYSTEM_NAME + "/ShooterRPM1";          //Number

    private static final String DBKEY_PWR2_INFO         = SUBSYSTEM_NAME + "/ShooterPwr2Info";      //String
    private static final String DBKEY_VEL2_INFO         = SUBSYSTEM_NAME + "/ShooterVel2Info";      //String
    private static final String DBKEY_VEL2_RPM          = SUBSYSTEM_NAME + "/ShooterRPM2";          //Number

    private static final String DBKEY_PAN_PWR_INFO      = SUBSYSTEM_NAME + "/PanPwrInfo";           //String
    private static final String DBKEY_PAN_POS_INFO      = SUBSYSTEM_NAME + "/PanPosInfo";           //String
    private static final String DBKEY_PAN_POS           = SUBSYSTEM_NAME + "/PanPos";               //Number

    private static final String DBKEY_TILT_PWR_INFO     = SUBSYSTEM_NAME + "/TiltPwrInfo";          //String
    private static final String DBKEY_TILT_POS_INFO     = SUBSYSTEM_NAME + "/TiltPosInfo";          //String
    private static final String DBKEY_TILT_POS          = SUBSYSTEM_NAME + "/TiltPos";              //Number

    private static final String DBKEY_LAUNCHER_POS      = SUBSYSTEM_NAME + "/LauncherPos";          //Number

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_PWR1_INFO, "");
        dashboard.refreshKey(DBKEY_VEL1_INFO, "");
        dashboard.refreshKey(DBKEY_VEL1_RPM, 0.0);

        dashboard.refreshKey(DBKEY_PWR2_INFO, "");
        dashboard.refreshKey(DBKEY_VEL2_INFO, "");
        dashboard.refreshKey(DBKEY_VEL2_RPM, 0.0);

        dashboard.refreshKey(DBKEY_PAN_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_PAN_POS_INFO, "");
        dashboard.refreshKey(DBKEY_PAN_POS, 0.0);

        dashboard.refreshKey(DBKEY_TILT_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_TILT_POS_INFO, "");
        dashboard.refreshKey(DBKEY_TILT_POS, 0.0);

        dashboard.refreshKey(DBKEY_LAUNCHER_POS, 0.0);
    }   //publishToDashboard

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
        if (slowLoop)
        {
            TrcMotor motor;

            dashboard.putString(
                DBKEY_PWR1_INFO, shooter.getShooterMotor1Power() + "/" + shooter.getShooterMotor1Current());
            dashboard.putString(
                DBKEY_VEL1_INFO, shooter.getShooterMotor1RPM() + "/" + shooter.getShooterMotor1TargetRPM());

            motor = shooter.getShooterMotor2();
            if (motor != null)
            {
                dashboard.putString(DBKEY_PWR2_INFO, motor.getPower() + "/" + motor.getCurrent());
                dashboard.putString(
                    DBKEY_VEL2_INFO, shooter.getShooterMotor2RPM() + "/" + shooter.getShooterMotor2TargetRPM());
            }

            motor = shooter.getPanMotor();
            if (motor != null)
            {
                dashboard.putString(DBKEY_PAN_PWR_INFO, motor.getPower() + "/" + motor.getCurrent());
                dashboard.putString(DBKEY_PAN_POS_INFO, motor.getPosition() + "/" + motor.getPidTarget());
            }

            motor = shooter.getTiltMotor();
            if (motor != null)
            {
                dashboard.putString(DBKEY_TILT_PWR_INFO, motor.getPower() + "/" + motor.getCurrent());
                dashboard.putString(DBKEY_TILT_POS_INFO, motor.getPosition() + "/" + motor.getPidTarget());
            }

            if (launcher != null)
            {
                dashboard.putNumber(DBKEY_LAUNCHER_POS, launcher.getPosition());
            }
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null)
        {
            if (tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, shooter.getShooterMotor1RPM());
            }
            else if (shooter.shooterMotor2 != null &&
                     tuneSubsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, shooter.getShooterMotor2RPM());
            }
            else if (shooter.panMotor != null && tuneSubsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, shooter.getPanAngle());
            }
            else if (shooter.tiltMotor != null && tuneSubsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, shooter.getTiltAngle());
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName)
    {
        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(ShooterMotorParams.motor1PidCoeffs)
                    .setFFCoefficients(ShooterMotorParams.motor1FFCoeffs)
                    .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, shooter1Velocity.getValue());
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(ShooterMotorParams.motor2PidCoeffs)
                    .setFFCoefficients(ShooterMotorParams.motor2FFCoeffs)
                    .setPidControlParams(ShooterMotorParams.PID_TOLERANCE, ShooterMotorParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, shooter2Velocity.getValue());
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(PanMotorParams.pidCoeffs)
                    .setPidControlParams(PanMotorParams.PID_TOLERANCE, PanMotorParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, PanMotorParams.FRONT_POS);
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(TiltMotorParams.pidCoeffs)
                    .setPidControlParams(TiltMotorParams.PID_TOLERANCE, TiltMotorParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, TiltMotorParams.MIN_POS);
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, LauncherParams.REST_POS);
        }
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     */
    @Override
    public void updateParamsFromDashboard(String subsystemName)
    {
        TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
        Double target = null;

        tuneSubsystemName = null;
        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, shooter1Velocity.getValue());
            shooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
            shooter1Velocity.setValue(target);
            shooter.setShooterMotorRPM(target, null);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, shooter2Velocity.getValue());
            shooter.shooterMotor2.setVelocityPidParameters(pidParams, null);
            shooter2Velocity.setValue(target);
            shooter.setShooterMotorRPM(null, target);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, PanMotorParams.FRONT_POS);
            shooter.panMotor.setPositionPidParameters(pidParams, null);
            shooter.panMotor.setPosition(instanceName, 0.0, target, true, null, null, 0.0);
            tuneSubsystemName = subsystemName;
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, TiltMotorParams.MIN_POS);
            shooter.tiltMotor.setPositionPidParameters(pidParams, null);
            shooter.tiltMotor.setPosition(instanceName, 0.0, target, true, null, null, 0.0);
            tuneSubsystemName = subsystemName;
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, LauncherParams.REST_POS);
            launcher.setPosition(target);
            tuneSubsystemName = subsystemName;
            pidParams = null;
        }

        if (target != null)
        {
            // Matched a subsystem name.
            shooter.tracer.traceInfo(
                instanceName, "Tune %s: pidParams=%s, target=%.3f", subsystemName, pidParams, target);
        }
    }   //updateParamsFromDashboard

    /**
     * This method is called to set the next tune target up from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetUp(String subsystemName)
    {
        Double target = null;

        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            target = shooter1Velocity.upValue();
            shooter.setShooterMotorRPM(target, null);
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            target = shooter2Velocity.upValue();
            shooter.setShooterMotorRPM(null, target);
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            target = shooter.panMotor.presetPositionUp(null, null);
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            target = shooter.tiltMotor.presetPositionUp(null, null);
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            target = LauncherParams.LAUNCH_POS;
            launcher.setPosition(target);
        }

        if (target != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, target);
            shooter.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetUp

    /**
     * This method is called to set the next tune target down from the current target.
     *
     * @param subsystemName specifies the name of the subsystem to update its tune target.
     */
    @Override
    public void setNextTuneTargetDown(String subsystemName)
    {
        Double target = null;

        if (subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR1_NAME))
        {
            target = shooter1Velocity.downValue();
            shooter.setShooterMotorRPM(target, null);
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(ShooterMotorParams.MOTOR2_NAME))
        {
            target = shooter2Velocity.downValue();
            shooter.setShooterMotorRPM(null, target);
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(PanMotorParams.MOTOR_NAME))
        {
            target = shooter.panMotor.presetPositionDown(null, null);
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(TiltMotorParams.MOTOR_NAME))
        {
            target = shooter.tiltMotor.presetPositionDown(null, null);
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(LauncherParams.SERVO_NAME))
        {
            target = LauncherParams.REST_POS;
            launcher.setPosition(target);
        }

        if (target != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, target);
            shooter.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class Shooter
