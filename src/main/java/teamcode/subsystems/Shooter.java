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

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = 10;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final double GOBILDA6000_CPR              = 28.0;
        public static final double GEAR_RATIO                   = 24.0/36.0;
        public static final double MOTOR_REV_PER_COUNT          = 1.0/(GOBILDA6000_CPR * GEAR_RATIO);
        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients shooterPidCoeffs =
            new TrcPidController.PidCoefficients(0.05, 0.0, 0.0, 0.008, 0.0);
        public static final double SHOOTER_PID_TOLERANCE        = 10.0;

        public static final double SHOOTER_MIN_VEL              = 10.0;     // in RPM
        public static final double SHOOTER_MAX_VEL              = 7360.0;   // in RPM
        public static final double SHOOTER_MIN_VEL_INC          = 1.0;      // in RPM
        public static final double SHOOTER_MAX_VEL_INC          = 100.0;    // in RPM
        public static final double SHOOTER_DEF_VEL              = 1000.0;   // in RPM
        public static final double SHOOTER_DEF_VEL_INC          = 10.0;     // in RPM
    }   //class Params

    private static final String DBKEY_SHOOTER_POWER             = Params.SUBSYSTEM_NAME + "/Power";
    private static final String DBKEY_SHOOTER_CURRENT           = Params.SUBSYSTEM_NAME + "/Current";
    private static final String DBKEY_SHOOTER_VELOCITY          = Params.SUBSYSTEM_NAME + "/Velocity";
    private static final String DBKEY_SHOOTER_TARGET_VEL        = Params.SUBSYSTEM_NAME + "/TargetVel";

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

        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS,
                Params.MOTOR_ENC_ABS, Params.MOTOR_INVERTED);
        shooter = new FrcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();
        TrcMotor shooterMotor = shooter.getShooterMotor1();
        shooterMotor.setPositionSensorScaleAndOffset(Params.MOTOR_REV_PER_COUNT, 0.0);
        shooterMotor.setVelocityPidParameters(Params.shooterPidCoeffs, Params.SHOOTER_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);

        shooterVelocity = new TrcDiscreteValue(
            Params.SUBSYSTEM_NAME + ".motorVel",
            Params.SHOOTER_MIN_VEL, Params.SHOOTER_MAX_VEL,
            Params.SHOOTER_MIN_VEL_INC, Params.SHOOTER_MAX_VEL_INC,
            Params.SHOOTER_DEF_VEL, Params.SHOOTER_DEF_VEL_INC);
        }   //Shooter

    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    public void setShooterEnabled(boolean enable)
    {
        if (enable)
        {
            shooter.setShooterMotorRPM(shooterVelocity.getValue(), 0.0);
        }
        else
        {
            shooter.stopShooter();
        }
    }   //setShooterEnabled

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
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Shooter does not support resetState.
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
     *        tuneParam6 - Shooter target velocity
     */
    @Override
    public void prepSubsystemForTuning(double... tuneParams)
    {
        shooter.getShooterMotor1().setVelocityPidParameters(
            tuneParams[0], tuneParams[1], tuneParams[2], tuneParams[3], tuneParams[4], tuneParams[5],
            Params.SOFTWARE_PID_ENABLED);
        shooterVelocity.setValue(tuneParams[6]);
    }   //prepSubsystemForTuning

}   //class Shooter
