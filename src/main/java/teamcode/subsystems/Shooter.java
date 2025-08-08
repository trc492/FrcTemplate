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

        public static final String MOTOR1_NAME                  = SUBSYSTEM_NAME + ".motor1";
        public static final int MOTOR1_ID                       = 10;
        public static final MotorType MOTOR1_TYPE               = MotorType.CanTalonSrx;
        public static final boolean MOTOR1_BRUSHLESS            = false;
        public static final boolean MOTOR1_ENC_ABS              = false;
        public static final boolean MOTOR1_INVERTED             = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = true;
        public static final String MOTOR2_NAME                  = SUBSYSTEM_NAME + ".motor2";
        public static final int MOTOR2_ID                       = 12;
        public static final MotorType MOTOR2_TYPE               = MotorType.CanTalonSrx;
        public static final boolean MOTOR2_BRUSHLESS            = false;
        public static final boolean MOTOR2_ENC_ABS              = false;
        public static final boolean MOTOR2_INVERTED             = true;

        public static final double GOBILDA1620_CPR              = ((1.0 + (46.0/17.0)) * 28.0);
        public static final double MOTOR_REV_PER_COUNT          = 1.0/GOBILDA1620_CPR;
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
    }   //class Params

    private static final String DBKEY_SHOOTER1_POWER            = Params.SUBSYSTEM_NAME + "/Power1";
    private static final String DBKEY_SHOOTER1_CURRENT          = Params.SUBSYSTEM_NAME + "/Current1";
    private static final String DBKEY_SHOOTER1_VELOCITY         = Params.SUBSYSTEM_NAME + "/Velocity1";
    private static final String DBKEY_SHOOTER2_POWER            = Params.SUBSYSTEM_NAME + "/Power2";
    private static final String DBKEY_SHOOTER2_CURRENT          = Params.SUBSYSTEM_NAME + "/Current2";
    private static final String DBKEY_SHOOTER2_VELOCITY         = Params.SUBSYSTEM_NAME + "/Velocity2";

    private final FrcDashboard dashboard;
    private final TrcShooter shooter;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_SHOOTER1_POWER, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER1_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER1_VELOCITY, "");
        dashboard.refreshKey(DBKEY_SHOOTER2_POWER, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER2_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_SHOOTER2_VELOCITY, "");

        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.MOTOR1_NAME, Params.MOTOR1_ID, Params.MOTOR1_TYPE, Params.MOTOR1_BRUSHLESS,
                Params.MOTOR1_ENC_ABS, Params.MOTOR1_INVERTED);
        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                Params.MOTOR1_NAME, Params.MOTOR2_ID, Params.MOTOR2_TYPE, Params.MOTOR2_BRUSHLESS,
                Params.MOTOR2_ENC_ABS, Params.MOTOR2_INVERTED);
        }
        shooter = new FrcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();
        configShooterMotor(shooter.getShooterMotor1(), Params.shooter1PidCoeffs);
        TrcMotor shooterMotor2 = shooter.getShooterMotor2();
        if (shooterMotor2 != null)
        {
            configShooterMotor(shooterMotor2, Params.shooter2PidCoeffs);
        }
    }   //Shooter

    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    /**
     * This method configures the shooter motor.
     *
     * @param motor specifies the motor to be configured.
     * @param pidCoeffs specifies the velocity PID control coefficients applied to the motor.
     */
    private void configShooterMotor(TrcMotor motor, TrcPidController.PidCoefficients pidCoeffs)
    {
        motor.setPositionSensorScaleAndOffset(Params.MOTOR_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(pidCoeffs, Params.SHOOTER_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
    }   //configShooterMotor

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
        dashboard.putNumber(DBKEY_SHOOTER1_POWER, shooter.getShooterMotor1Power());
        dashboard.putNumber(DBKEY_SHOOTER1_CURRENT, shooter.getShooterMotor1Current());
        dashboard.putString(
            DBKEY_SHOOTER1_VELOCITY,
            String.format("%.1f/%.1f", shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM()));
        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            dashboard.putNumber(DBKEY_SHOOTER2_POWER, shooter.getShooterMotor2Power());
            dashboard.putNumber(DBKEY_SHOOTER2_CURRENT, shooter.getShooterMotor2Current());
            dashboard.putString(
                DBKEY_SHOOTER2_VELOCITY,
                String.format("%.1f/%.1f", shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM()));
        }
        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param pidCoeffs specifies the PID coefficients for the subsystem.
     * @param pidTolerance specifies the PID tolerance.
     * @param gravityCompPower specifies the gravity compensation power for the subsystem (not used).
     */
    @Override
    public void prepSubsystemForTuning(
        TrcPidController.PidCoefficients pidCoeffs, double pidTolerance, double gravityCompPower)
    {
        shooter.getShooterMotor1().setPositionPidParameters(pidCoeffs, pidTolerance, true);
        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooter.getShooterMotor2().setPositionPidParameters(pidCoeffs, pidTolerance, true);
        }
    }   //prepSubsystemForTuning

}   //class Shooter
