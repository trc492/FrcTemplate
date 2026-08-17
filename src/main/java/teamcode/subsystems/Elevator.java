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
import teamcode.FrcTest;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Elevator Subsystem. This implementation consists of one or two motors with built-in
 * encoders and optional limit switches for zero calibrating the relative encoder. If there is no lower limit
 * switch, it will use motor stall detection to zero calibrate the built-in relative encoder. It supports gravity
 * compensation. In the case of an elevator, gravity compensation power is the constant power required to hold the
 * elevator at any position.
 */
public class Elevator extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Elevator";
    private static final boolean NEED_ZERO_CAL = true;

    public static final class Params
    {
        public static final boolean HAS_TWO_MOTORS              = false;
        public static final boolean HAS_LOWER_LIMIT_SWITCH      = false;
        public static final boolean HAS_UPPER_LIMIT_SWITCH      = false;

        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final int PRIMARY_MOTOR_ID                = 10;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final int FOLLOWER_MOTOR_ID               = 12;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int LOWER_LIMIT_SWITCH_CHANNEL      = 0;
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final String UPPER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int UPPER_LIMIT_SWITCH_CHANNEL      = 1;
        public static final boolean UPPER_LIMIT_SWITCH_INVERTED = false;

        public static final double POS_OFFSET                   = 13.5;
        public static final double INCHES_PER_COUNT             = (32.125 - POS_OFFSET) / 5024.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.25;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 32.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 20.0, 25.0, 30.0, MAX_POS};
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
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor motor;
    private Double tuneGravityCompPower = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.MOTOR_TYPE, Params.PRIMARY_MOTOR_INVERTED, true, true,
                Params.PRIMARY_MOTOR_ID, null, null)
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);

        if (Params.HAS_TWO_MOTORS)
        {
            motorParams.addFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.MOTOR_TYPE, Params.FOLLOWER_MOTOR_INVERTED,
                Params.FOLLOWER_MOTOR_ID, null, null);
        }

        if (Params.HAS_LOWER_LIMIT_SWITCH)
        {
            motorParams.setLowerLimitSwitch(
                Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_CHANNEL, Params.LOWER_LIMIT_SWITCH_INVERTED);
        }

        if (Params.HAS_UPPER_LIMIT_SWITCH)
        {
            motorParams.setUpperLimitSwitch(
                Params.UPPER_LIMIT_SWITCH_NAME, Params.UPPER_LIMIT_SWITCH_CHANNEL, Params.UPPER_LIMIT_SWITCH_INVERTED);
        }

        motor = new FrcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED), null);
        motor.setPositionPidPowerComp(this::getGravityComp);

        if (!Params.HAS_LOWER_LIMIT_SWITCH)
        {
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
        }
    }   //Elevator

    /**
     * This method returns the created Elevator motor.
     *
     * @return created elevator motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method calculates the power required to make the elevator gravity neutral.
     *
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getGravityComp(TrcMotor motor, double currPower)
    {
        return tuneGravityCompPower != null? tuneGravityCompPower: Params.GRAVITY_COMP_POWER;
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
        motor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, completionEvent, Params.ZERO_CAL_TIMEOUT);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        motor.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
    }   //resetState

    private static final String DBKEY_PWR_INFO          = SUBSYSTEM_NAME + "/PwrInfo";      //String
    private static final String DBKEY_POS_INFO          = SUBSYSTEM_NAME + "/PosInfo";      //String
    private static final String DBKEY_POS               = SUBSYSTEM_NAME + "/Pos";          //Number
    private static final String DBKEY_LOWER_LIMIT       = SUBSYSTEM_NAME + "/LowerLimit";   //Boolean
    private static final String DBKEY_UPPER_LIMIT       = SUBSYSTEM_NAME + "/UpperLimit";   //Boolean

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_POS_INFO, "");
        dashboard.refreshKey(DBKEY_POS, 0.0);
        dashboard.refreshKey(DBKEY_LOWER_LIMIT, false);
        dashboard.refreshKey(DBKEY_UPPER_LIMIT, false);
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
            dashboard.putString(DBKEY_PWR_INFO, motor.getPower() + "/" + motor.getCurrent());
            dashboard.putString(DBKEY_POS_INFO, motor.getPosition() + "/" + motor.getPidTarget());

            if (Params.HAS_LOWER_LIMIT_SWITCH)
            {
                dashboard.putBoolean(DBKEY_LOWER_LIMIT, motor.isLowerLimitSwitchActive());
            }

            if (Params.HAS_UPPER_LIMIT_SWITCH)
            {
                dashboard.putBoolean(DBKEY_UPPER_LIMIT, motor.isUpperLimitSwitchActive());
            }
        }
        dashboard.putNumber(DBKEY_POS, motor.getPosition());

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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_MOTOR_NAME))
        {
            double currValue = motor.getPosition();
            double paramValue = nextValueUp == null? currValue:
                motor.getNextPresetPosition(currValue, nextValueUp);

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.posPidCoeffs)
                    .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED)
                    .setTuningParams(paramValue));

            if (tuneGravityCompPower == null)
            {
                tuneGravityCompPower = dashboard.getNumber(
                    FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.GRAVITY_COMP_POWER);
            }
            dashboard.putNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, tuneGravityCompPower);
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

            motor.setPositionPidParameters(pidParams, null);
            tuneGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.GRAVITY_COMP_POWER);
            motor.tracer.traceInfo(
                instanceName, "Tune %s: PidParams=%s, GravityPower=%.3f",
                subsystemName, pidParams, tuneGravityCompPower);
        }
    }   //updateParamsFromDashboard

}   //class Elevator
