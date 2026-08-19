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
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcEncoder.EncoderType;
import teamcode.FrcTest;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a CrServoArm Subsystem. This implementation consists of two Axon servos running in Continuous
 * Rotation mode with an analog absolute encoder. It does not require zero calibration. Therefore, limit switches are
 * optional. If using limit switches, they are for movement range protection. If not using limit switches, software
 * limit must be set. It supports gravity compensation by computing the power required to hold the arm at its current
 * angle.
 */
public class CrServoArm extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "CrServoArm";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        public static final MotorType MOTOR_TYPE                = MotorType.CRServo;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final boolean PRIMARY_MOTOR_INVERTED      = false;
        public static final boolean PRIMARY_MOTOR_VOLTCOMP_ENABLED = false;
        public static final boolean PRIMARY_MOTOR_BRAKE_ENABLED = false;
        public static final int PRIMARY_MOTOR_CHANNEL           = 0;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;
        public static final boolean FOLLOWER_MOTOR_VOLTCOMP_ENABLED = false;
        public static final boolean FOLLOWER_MOTOR_BRAKE_ENABLED = false;
        public static final int FOLLOWER_MOTOR_CHANNEL          = 1;

        public static final String ABSENC_NAME                  = SUBSYSTEM_NAME + ".absEnc";
        public static final EncoderType ABSENC_TYPE             = EncoderType.AnalogEncoder;
        public static final boolean ABSENC_INVERTED             = true;
        public static final int ABSENC_CHANNEL                  = 0;

        public static final double POS_DEG_SCALE                = 360.0;
        public static final double POS_OFFSET                   = 27.0;
        public static final double ABSENC_ZERO_OFFSET           = 0.949697;
        public static final double POWER_LIMIT                  = 0.25;

        public static final double MIN_POS                      = 27.3;
        public static final double MAX_POS                      = 300.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {
            30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.0162, 0.0, 0.0, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_POWER           = 0.1675;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor motor;
    private Double tuneTarget = null;
    private Double tuneGravityCompPower = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CrServoArm()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED,
                Params.PRIMARY_MOTOR_VOLTCOMP_ENABLED, Params.PRIMARY_MOTOR_BRAKE_ENABLED,
                Params.PRIMARY_MOTOR_CHANNEL, null, null)
            .addFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED,
                Params.FOLLOWER_MOTOR_CHANNEL, null, null)
            .setExternalEncoder(
                Params.ABSENC_NAME, Params.ABSENC_TYPE, Params.ABSENC_INVERTED, Params.ABSENC_CHANNEL)
            .setPositionScaleAndOffset(Params.POS_DEG_SCALE, Params.POS_OFFSET, Params.ABSENC_ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FrcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED), null);
        motor.setPositionPidPowerComp(this::getGravityComp);
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //CrServoArm

    /**
     * This method returns the created CrServoArm motor.
     *
     * @return created arm motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method calculates the power required to make the arm gravity neutral.
     *
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneGravityCompPower != null? tuneGravityCompPower: Params.GRAVITY_COMP_POWER;
        return gravityCompPower * Math.sin(Math.toRadians(motor.getPosition()));
    }   //getGravityComp

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        motor.cancel();
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
        // No zero calibration needed for absolute encoder.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        motor.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
    }   //resetState

    private static final String DBKEY_POWER             = SUBSYSTEM_NAME + "/Power";        //Number
    private static final String DBKEY_POS_INFO          = SUBSYSTEM_NAME + "/PosInfo";      //String

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_POWER, 0.0);
        dashboard.refreshKey(DBKEY_POS_INFO, "");
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
            dashboard.putNumber(DBKEY_POWER, motor.getPower());
            dashboard.putString(DBKEY_POS_INFO, motor.getPosition() + "/" + motor.getPidTarget());
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, motor.getPosition());
        dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, motor.getPidTarget());

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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.posPidCoeffs)
                    .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED));

            if (tuneTarget != null)
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, tuneTarget);
            }
            else
            {
                tuneTarget = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.MIN_POS);
            }

            if (tuneGravityCompPower != null)
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, tuneGravityCompPower);
            }
            else
            {
                tuneGravityCompPower = dashboard.getNumber(
                    FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.GRAVITY_COMP_POWER);
            }
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();

            tuneTarget = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.MIN_POS);
            tuneGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.GRAVITY_COMP_POWER);
            motor.setPositionPidParameters(pidParams, null);
            motor.setPosition(tuneTarget);
            motor.tracer.traceInfo(
                instanceName, "Tune %s: PidParams=%s, target=%.3f, GravityPower=%.3f",
                subsystemName, pidParams, tuneTarget, tuneGravityCompPower);
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            tuneTarget = motor.presetPositionUp(null, null);
            motor.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, tuneTarget);
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            tuneTarget = motor.presetPositionDown(null, null);
            motor.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, tuneTarget);
        }
    }   //setNextTuneTargetDown

}   //class CrServoArm
