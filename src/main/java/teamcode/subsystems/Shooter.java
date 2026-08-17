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
    public static final String SUBSYSTEM_NAME                   = "Shooter";
    public static final boolean NEED_ZERO_CAL                   = false;

    public static final class Params
    {
        public static final boolean HAS_TWO_SHOOTER_MOTORS      = false;
        public static final boolean HAS_PAN_MOTOR               = false;
        public static final boolean HAS_TILT_MOTOR              = false;
        public static final boolean HAS_LAUNCHER                = false;

        // Shooter Motor1
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final String SHOOTER_MOTOR1_NAME          = SUBSYSTEM_NAME + ".shooterMotor1";
        public static final int SHOOTER_MOTOR1_ID               = 10;
        public static final boolean SHOOTER_MOTOR1_INVERTED     = true;

        // Shooter Motor2
        public static final String SHOOTER_MOTOR2_NAME          = SUBSYSTEM_NAME + ".shooterMotor2";
        public static final int SHOOTER_MOTOR2_ID               = 12;
        public static final boolean SHOOTER_MOTOR2_INVERTED     = true;

        // Shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final double GOBILDA6000_CPR              = 28.0;
        public static final double SHOOTER_GEAR_RATIO           = 24.0/36.0;
        public static final double SHOOTER_REV_PER_COUNT        = 1.0/(GOBILDA6000_CPR * SHOOTER_GEAR_RATIO);
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= true;
        public static final TrcPidController.PidCoefficients shooter1PidCoeffs =
            new TrcPidController.PidCoefficients(0.075, 0.0, 0.0, 0.0, 0.0);
        public static final TrcPidController.FFCoefficients shooter1FFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.008, 0.0);
        public static final TrcPidController.PidCoefficients shooter2PidCoeffs =
            new TrcPidController.PidCoefficients(0.075, 0.0, 0.0, 0.008, 0.0);
        public static final TrcPidController.FFCoefficients shooter2FFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.008, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 1.0;      // in RPS (60 RPM)
        public static final double SHOOTER_OFF_DELAY            = 0.5;      // in sec

        // These are for tuning shooter motor with gamepad.
        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 7360.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 100.0;    // in RPM

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".panMotor";
        public static final int PAN_MOTOR_ID                    = 14;
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.CanTalonSrx;
        public static final boolean PAN_MOTOR_BRUSHLESS         = false;
        public static final boolean PAN_MOTOR_ENC_ABS           = false;
        public static final boolean PAN_MOTOR_INVERTED          = false;

        public static final double PAN_ZERO_CAL_POWER           = -0.2;
        public static final double PAN_STALL_MIN_POWER          = Math.abs(PAN_ZERO_CAL_POWER);
        public static final double PAN_STALL_TOLERANCE          = 0.1;
        public static final double PAN_STALL_TIMEOUT            = 0.1;
        public static final double PAN_STALL_RESET_TIMEOUT      = 0.0;

        public static final double PAN_DEG_PER_COUNT            = 1.0;
        public static final double PAN_POS_OFFSET               = -90.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;
        public static final TrcPidController.PidCoefficients panPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double PAN_PID_TOLERANCE            = 1.0;

        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -90.0;
        public static final double PAN_MAX_POS                  = 90.0;

        // Tilt Motor
        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".tiltMotor";
        public static final int TILT_MOTOR_ID                   = 16;
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanTalonSrx;
        public static final boolean TILT_MOTOR_BRUSHLESS        = false;
        public static final boolean TILT_MOTOR_ENC_ABS          = false;
        public static final boolean TILT_MOTOR_INVERTED         = false;

        public static final double TILT_DEG_PER_COUNT           = 1.0;
        public static final double TILT_POS_OFFSET              = 0.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;
        public static final TrcPidController.PidCoefficients tiltPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double TILT_PID_TOLERANCE           = 1.0;

        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = 0.0;
        public static final double TILT_MAX_POS                 = 90.0;

        public static final TrcPose2D robotToShooterPose        = new TrcPose2D(0.0, 0.0, 0.0);

        public static final TrcLookupTable.Region[] regions =
        {
            new TrcLookupTable.Region(60.0, new double[][] {null})
        };

        public static final TrcLookupTable shootParamTable = new TrcLookupTable()
            //        name,                 distance,   region,             ShooterVel
            .addEntry(null,                 36.0,       regions[0],         60.0)
            .addEntry(null,                 48.0,       regions[0],         70.0)
            .addEntry(null,                 60.0,       regions[0],         80.0)
            .addEntry(null,                 72.0,       regions[0],         90.0);

        // Launcher
        public static final String LAUNCHER_SERVO_NAME          = SUBSYSTEM_NAME + ".launcher";
        public static final int LAUNCHER_SERVO_CHANNEL          = 0;
        public static final boolean LAUNCHER_SERVO_INVERTED     = true;
        public static double LAUNCHER_REST_POS                  = 0.0;
        public static double LAUNCHER_LAUNCH_POS                = 0.5;
        public static double LAUNCHER_LAUNCH_DURATION           = 0.5;  // in seconds
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcShooter shooter;
    public final TrcDiscreteValue shooter1Velocity;
    public final TrcDiscreteValue shooter2Velocity;
    public final TrcServo launcher;
    private String launchOwner;
    private TrcEvent launchCompletionEvent;
    private TrcEvent launchCallbackEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.SHOOTER_MOTOR1_NAME, Params.SHOOTER_MOTOR_TYPE, Params.SHOOTER_MOTOR1_INVERTED,
                Params.SHOOTER_MOTOR1_ID, null, null, false);

        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                Params.SHOOTER_MOTOR2_NAME, Params.SHOOTER_MOTOR_TYPE, Params.SHOOTER_MOTOR2_INVERTED,
                Params.SHOOTER_MOTOR2_ID, null, null, false, true);
        }

        if (Params.HAS_PAN_MOTOR)
        {
            shooterParams.setPanMotor(
                Params.PAN_MOTOR_NAME, Params.PAN_MOTOR_TYPE, Params.PAN_MOTOR_INVERTED, Params.PAN_MOTOR_ID,
                null, null,
                new TrcShooter.PanTiltParams(Params.PAN_POWER_LIMIT, Params.PAN_MIN_POS, Params.PAN_MAX_POS));
        }

        if (Params.HAS_TILT_MOTOR)
        {
            shooterParams.setTiltMotor(
                Params.TILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.TILT_MOTOR_INVERTED, Params.TILT_MOTOR_ID,
                null, null, 
                new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS));
        }

        shooter = new FrcShooter(SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(Params.SHOOTER_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(Params.shooter1PidCoeffs)
                .setFFCoefficients(Params.shooter1FFCoeffs)
                .setPidControlParams(Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED), null);
        // For tuning shooter motor 1 PID.
        shooter1Velocity = new TrcDiscreteValue(
            SUBSYSTEM_NAME + ".motor1TargetVel",
            Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
            Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
            Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(Params.SHOOTER_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.shooter2PidCoeffs)
                    .setFFCoefficients(Params.shooter2FFCoeffs)
                    .setPidControlParams(Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED), null);
            // For tuning shooter motor 2 PID.
            shooter2Velocity = new TrcDiscreteValue(
                SUBSYSTEM_NAME + ".motor2TargetVel",
                Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
                Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
                Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);
        }
        else
        {
            shooter2Velocity = null;
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(Params.PAN_DEG_PER_COUNT, Params.PAN_POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.panPidCoeffs)
                    .setPidControlParams(Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED),
                null);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                Params.PAN_STALL_MIN_POWER, Params.PAN_STALL_TOLERANCE, Params.PAN_STALL_TIMEOUT,
                Params.PAN_STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(Params.PAN_MIN_POS, Params.PAN_MAX_POS, false);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(Params.TILT_DEG_PER_COUNT, Params.TILT_POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.tiltPidCoeffs)
                    .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED),
                null);
            motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);
        }

        if (Params.HAS_LAUNCHER)
        {
            FrcServoActuator.Params launcherParams = new FrcServoActuator.Params()
                .setPrimaryServo(
                    Params.LAUNCHER_SERVO_NAME, Params.LAUNCHER_SERVO_CHANNEL, Params.LAUNCHER_SERVO_INVERTED);
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
        if (launcher != null)
        {
            TrcDbgTrace.globalTraceInfo(instanceName, "shoot(owner=" + owner + ", event=" + completionEvent + ")");
            launchOwner = owner;
            launchCompletionEvent = completionEvent;
            launchCallbackEvent = new TrcEvent(SUBSYSTEM_NAME + ".launchCallback");
            launchCallbackEvent.setCallback(this::launchCallback, null);
            launcher.setPosition(
                owner, 0.0, Params.LAUNCHER_LAUNCH_POS, launchCallbackEvent, Params.LAUNCHER_LAUNCH_DURATION);
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
        launcher.setPosition(launchOwner, 0.0, Params.LAUNCHER_REST_POS, null, 0.0);
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
        shooter.panMotor.zeroCalibrate(owner, Params.PAN_ZERO_CAL_POWER, completionEvent, 0.0);
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

            if (shooter.getShooterMotor2() != null)
            {
                dashboard.putString(
                    DBKEY_PWR2_INFO, shooter.getShooterMotor2Power() + "/" + shooter.getShooterMotor2Current());
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
        }

        dashboard.putNumber(DBKEY_VEL1_RPM, shooter.getShooterMotor1RPM());

        if (shooter.shooterMotor2 != null)
        {
            dashboard.putNumber(DBKEY_VEL2_RPM, shooter.getShooterMotor2RPM());
        }

        if (shooter.panMotor != null)
        {
            dashboard.putNumber(DBKEY_PAN_POS, shooter.getPanAngle());
        }

        if (shooter.tiltMotor != null)
        {
            dashboard.putNumber(DBKEY_TILT_POS, shooter.getTiltAngle());
        }

        if (launcher != null)
        {
            dashboard.putNumber(DBKEY_LAUNCHER_POS, launcher.getPosition());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     *
     * @param subsystemName specifies the name of the subsystem to be updated.
     * @param nextValueUp specifies true for the next preset target value up, false for next preset target value down,
     *        null for the current target value.
     */
    @Override
    public void updateParamsToDashboard(String subsystemName, Boolean nextValueUp)
    {
        if (subsystemName.equalsIgnoreCase(Params.SHOOTER_MOTOR1_NAME))
        {
            if (nextValueUp != null)
            {
                if (nextValueUp)
                {
                    shooter1Velocity.upValue();
                }
                else
                {
                    shooter1Velocity.downValue();
                }
            }

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.shooter1PidCoeffs)
                    .setFFCoefficients(Params.shooter1FFCoeffs)
                    .setPidControlParams(Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED)
                    .setTuningParams(shooter1Velocity.getValue()));
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(Params.SHOOTER_MOTOR2_NAME))
        {
            if (nextValueUp != null)
            {
                if (nextValueUp)
                {
                    shooter2Velocity.upValue();
                }
                else
                {
                    shooter2Velocity.downValue();
                }
            }

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.shooter2PidCoeffs)
                    .setFFCoefficients(Params.shooter2FFCoeffs)
                    .setPidControlParams(Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED)
                    .setTuningParams(shooter2Velocity.getValue()));
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(Params.PAN_MOTOR_NAME))
        {
            double currValue = shooter.panMotor.getPosition();
            double paramValue = nextValueUp == null? currValue:
                shooter.panMotor.getNextPresetPosition(currValue, nextValueUp);

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.panPidCoeffs)
                    .setPidControlParams(Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED)
                    .setTuningParams(paramValue));
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(Params.TILT_MOTOR_NAME))
        {
            double currValue = shooter.tiltMotor.getPosition();
            double paramValue = nextValueUp == null? currValue:
                shooter.tiltMotor.getNextPresetPosition(currValue, nextValueUp);

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.tiltPidCoeffs)
                    .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED)
                    .setTuningParams(paramValue));
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(Params.LAUNCHER_SERVO_NAME))
        {
            double currValue = launcher.getPosition();
            double paramValue = nextValueUp == null? currValue:
                   nextValueUp? Params.LAUNCHER_LAUNCH_POS: Params.LAUNCHER_REST_POS;

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams().setTuningParams(paramValue));
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
        boolean foundMatch = false;

        if (subsystemName.equalsIgnoreCase(Params.SHOOTER_MOTOR1_NAME))
        {
            shooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
            shooter.setShooterMotorRPM(pidParams.pidTarget, null);
            foundMatch = true;
        }
        else if (shooter.shooterMotor2 != null && subsystemName.equalsIgnoreCase(Params.SHOOTER_MOTOR2_NAME))
        {
            shooter.shooterMotor2.setVelocityPidParameters(pidParams, null);
            shooter.setShooterMotorRPM(null, pidParams.pidTarget);
            foundMatch = true;
        }
        else if (shooter.panMotor != null && subsystemName.equalsIgnoreCase(Params.PAN_MOTOR_NAME))
        {
            shooter.panMotor.setPositionPidParameters(pidParams, null);
            shooter.setPanAngle(pidParams.pidTarget);
            foundMatch = true;
        }
        else if (shooter.tiltMotor != null && subsystemName.equalsIgnoreCase(Params.TILT_MOTOR_NAME))
        {
            shooter.tiltMotor.setPositionPidParameters(pidParams, null);
            shooter.setTiltAngle(pidParams.pidTarget);
            foundMatch = true;
        }
        else if (launcher != null && subsystemName.equalsIgnoreCase(Params.LAUNCHER_SERVO_NAME))
        {
            launcher.setPosition(pidParams.pidTarget);
            foundMatch = true;
        }

        if (foundMatch)
        {
            shooter.tracer.traceInfo(instanceName, "Tune %s: target=%f", subsystemName, pidParams.pidTarget);
        }
    }   //updateParamsFromDashboard

}   //class Shooter
