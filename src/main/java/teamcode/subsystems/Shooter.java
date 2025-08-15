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
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcShooter;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcDiscreteValue;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Shooter Subsystem.
 */
public class Shooter extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final boolean NEED_ZERO_CAL               = false;
        public static final boolean HAS_PAN_MOTOR               = false;
        public static final boolean HAS_TILT_MOTOR              = false;

        // Shooter Motor
        public static final String SHOOTER_MOTOR_NAME           = SUBSYSTEM_NAME + ".shooterMotor";
        public static final int SHOOTER_MOTOR_ID                = 10;
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean SHOOTER_MOTOR_BRUSHLESS     = false;
        public static final boolean SHOOTER_MOTOR_ENC_ABS       = false;
        public static final boolean SHOOTER_MOTOR_INVERTED      = true;

        public static final double GOBILDA6000_CPR              = 28.0;
        public static final double SHOOTER_GEAR_RATIO           = 24.0/36.0;
        public static final double SHOOTER_REV_PER_COUNT        = 1.0/(GOBILDA6000_CPR * SHOOTER_GEAR_RATIO);
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= true;
        public static final TrcPidController.PidCoefficients shooterPidCoeffs =
            new TrcPidController.PidCoefficients(0.05, 0.0, 0.0, 0.008, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 10.0;

        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 7360.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 10.0;     // in RPM

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".panMotor";
        public static final int PAN_MOTOR_ID                    = 12;
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.CanTalonSrx;
        public static final boolean PAN_MOTOR_BRUSHLESS         = false;
        public static final boolean PAN_MOTOR_ENC_ABS           = false;
        public static final boolean PAN_MOTOR_INVERTED          = false;

        public static final double PAN_DEG_PER_COUNT            = 1.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;
        public static final TrcPidController.PidCoefficients panPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double PAN_PID_TOLERANCE            = 1.0;

        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -90.0;
        public static final double PAN_MAX_POS                  = 90.0;

        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".tiltMotor";
        public static final int TILT_MOTOR_ID                   = 14;
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanTalonSrx;
        public static final boolean TILT_MOTOR_BRUSHLESS        = false;
        public static final boolean TILT_MOTOR_ENC_ABS          = false;
        public static final boolean TILT_MOTOR_INVERTED         = false;

        public static final double TILT_DEG_PER_COUNT           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;
        public static final TrcPidController.PidCoefficients tiltPidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        public static final double TILT_PID_TOLERANCE           = 1.0;

        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = 0.0;
        public static final double TILT_MAX_POS                 = 90.0;
    }   //class Params

    private static final String DBKEY_SHOOTER_POWER             = Params.SUBSYSTEM_NAME + "/ShooterPower";
    private static final String DBKEY_SHOOTER_CURRENT           = Params.SUBSYSTEM_NAME + "/ShooterCurrent";
    private static final String DBKEY_SHOOTER_VELOCITY          = Params.SUBSYSTEM_NAME + "/ShooterVelocity";
    private static final String DBKEY_SHOOTER_TARGET_VEL        = Params.SUBSYSTEM_NAME + "/ShooterTargetVel";

    private static final String DBKEY_PAN_POWER                 = Params.SUBSYSTEM_NAME + "/PanPower";
    private static final String DBKEY_PAN_CURRENT               = Params.SUBSYSTEM_NAME + "/PanCurrent";
    private static final String DBKEY_PAN_POS                   = Params.SUBSYSTEM_NAME + "/PanPos";
    private static final String DBKEY_PAN_TARGET_POS            = Params.SUBSYSTEM_NAME + "/PanTargetPos";

    private static final String DBKEY_TILT_POWER                = Params.SUBSYSTEM_NAME + "/TiltPower";
    private static final String DBKEY_TILT_CURRENT              = Params.SUBSYSTEM_NAME + "/TiltCurrent";
    private static final String DBKEY_TILT_POS                  = Params.SUBSYSTEM_NAME + "/TiltPos";
    private static final String DBKEY_TILT_TARGET_POS           = Params.SUBSYSTEM_NAME + "/TiltTargetPos";

    private final FrcDashboard dashboard;
    private final TrcShooter shooter;
    public final TrcDiscreteValue shooterVelocity;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();

        dashboard.refreshKey(DBKEY_SHOOTER_POWER, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER_VELOCITY, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER_TARGET_VEL, 0.0);

        dashboard.refreshKey(DBKEY_PAN_POWER, 0.0);
        dashboard.refreshKey(DBKEY_PAN_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_PAN_POS, 0.0);
        dashboard.refreshKey(DBKEY_PAN_TARGET_POS, 0.0);

        dashboard.refreshKey(DBKEY_TILT_POWER, 0.0);
        dashboard.refreshKey(DBKEY_TILT_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_TILT_POS, 0.0);
        dashboard.refreshKey(DBKEY_TILT_TARGET_POS, 0.0);

        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.SHOOTER_MOTOR_NAME, Params.SHOOTER_MOTOR_ID, Params.SHOOTER_MOTOR_TYPE,
                Params.SHOOTER_MOTOR_BRUSHLESS, Params.SHOOTER_MOTOR_ENC_ABS, Params.SHOOTER_MOTOR_INVERTED);

        if (Params.HAS_PAN_MOTOR)
        {
            shooterParams.setPanMotor(
                Params.PAN_MOTOR_NAME, Params.PAN_MOTOR_ID, Params.PAN_MOTOR_TYPE, Params.PAN_MOTOR_BRUSHLESS,
                Params.PAN_MOTOR_ENC_ABS, Params.PAN_MOTOR_INVERTED,
                new TrcShooter.PanTiltParams(Params.PAN_POWER_LIMIT, Params.PAN_MIN_POS, Params.PAN_MAX_POS));
        }

        if (Params.HAS_TILT_MOTOR)
        {
            shooterParams.setTiltMotor(
                Params.TILT_MOTOR_NAME, Params.TILT_MOTOR_ID, Params.TILT_MOTOR_TYPE, Params.TILT_MOTOR_BRUSHLESS,
                Params.TILT_MOTOR_ENC_ABS, Params.TILT_MOTOR_INVERTED,
                new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS));
        }

        shooter = new FrcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor shooterMotor = shooter.getShooterMotor1();
        shooterMotor.setPositionSensorScaleAndOffset(Params.SHOOTER_REV_PER_COUNT, 0.0);
        shooterMotor.setVelocityPidParameters(
            Params.shooterPidCoeffs, Params.SHOOTER_PID_TOLERANCE, Params.SHOOTER_SOFTWARE_PID_ENABLED);

        shooterVelocity = new TrcDiscreteValue(
            Params.SUBSYSTEM_NAME + ".motorTargetVel",
            Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
            Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
            Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);

        if (Params.HAS_PAN_MOTOR)
        {
            TrcMotor panMotor = shooter.getPanMotor();
            panMotor.setPositionSensorScaleAndOffset(Params.PAN_DEG_PER_COUNT, 0.0);
            panMotor.setPositionPidParameters(
                Params.panPidCoeffs, Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED);
        }

        if (Params.HAS_TILT_MOTOR)
        {
            TrcMotor tiltMotor = shooter.getTiltMotor();
            tiltMotor.setPositionSensorScaleAndOffset(Params.TILT_DEG_PER_COUNT, 0.0);
            tiltMotor.setPositionPidParameters(
                Params.tiltPidCoeffs, Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED);
        }
    }   //Shooter

    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

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
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // Shooter does not need zero calibration.
        // Assuming pan and tilt are using absolute encoders. If not, we need to add code to zero calibrate pan and
        // tilt here.
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

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.putNumber(DBKEY_SHOOTER_POWER, shooter.getShooterMotor1Power());
        dashboard.putNumber(DBKEY_SHOOTER_CURRENT, shooter.getShooterMotor1Current());
        dashboard.putNumber(DBKEY_SHOOTER_VELOCITY, shooter.getShooterMotor1RPM());
        dashboard.putNumber(DBKEY_SHOOTER_TARGET_VEL, shooter.getShooterMotor1TargetRPM());

        TrcMotor motor = shooter.getPanMotor();
        dashboard.putNumber(DBKEY_PAN_POWER, motor.getPower());
        dashboard.putNumber(DBKEY_PAN_CURRENT, motor.getCurrent());
        dashboard.putNumber(DBKEY_PAN_POS, motor.getPosition());
        dashboard.putNumber(DBKEY_PAN_TARGET_POS, motor.getPidTarget());

        motor = shooter.getTiltMotor();
        dashboard.putNumber(DBKEY_TILT_POWER, motor.getPower());
        dashboard.putNumber(DBKEY_TILT_CURRENT, motor.getCurrent());
        dashboard.putNumber(DBKEY_TILT_POS, motor.getPosition());
        dashboard.putNumber(DBKEY_TILT_TARGET_POS, motor.getPidTarget());

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param tuneParams specifies tuning parameters.
     *        tuneParam0 - Kp
     *        tuneParam1 - Ki
     *        tuneParam2 - Kd
     *        tuneParam3 - Kf
     *        tuneParam4 - iZone
     *        tuneParam5 - PidTolerance
     *        tuneParam6 - Shooter motor target velocity
     */
    @Override
    public void prepSubsystemForTuning(double... tuneParams)
    {
        // To tune Shooter Motor PID, uncomment the code below.
        shooter.getShooterMotor1().setVelocityPidParameters(
            tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
            Params.SHOOTER_SOFTWARE_PID_ENABLED);
        shooterVelocity.setValue(tuneParams[6]);

        // To tune Pan Motor PID, uncomment the code below.
        // shooter.getPanMotor().setPositionPidParameters(
        //     tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
        //     Params.PAN_SOFTWARE_PID_ENABLED);

        // To tune Tilt Motor PID, uncomment the code below.
        // shooter.getTiltMotor().setPositionPidParameters(
        //     tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
        //     Params.TILT_SOFTWARE_PID_ENABLED);
    }   //prepSubsystemForTuning

}   //class Shooter
