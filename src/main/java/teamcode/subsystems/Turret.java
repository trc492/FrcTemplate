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
 * This class implements a Turret Subsystem. This implementation consists of a motor with built-in encoder. It has
 * a lower limit switch for zero calibrating the built-in relative encoder. Since Turret is circular in nature, it
 * is recommended to implement a hard stop to prevent the Turret from overrunning the upper limit causing the wiring
 * harness to be twisted. Even though we do implement soft limits on the Turret, hard stop would prevent folks from
 * spinning the turret round and round twisting the wiring harness when the robot is off.
 */
public class Turret extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME                   = "Turret";
    public static final boolean NEED_ZERO_CAL                   = true;

    public static final class Params
    {
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = 10;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String LOWER_LIMIT_SWITCH_NAME      = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int LOWER_LIMIT_SWITCH_CHANNEL      = 0;
        public static final boolean LOWER_LIMIT_SWITCH_INVERTED = false;

        public static final double ENCODER_PPR                  = 288.0;
        public static final double GEAR_RATIO                   = 100.0/60.0;
        public static final double DEG_PER_COUNT                = 360.0/(ENCODER_PPR*GEAR_RATIO);
        public static final double POS_OFFSET                   = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.3;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 325.0;
        public static final double BACK                         = 0.0;
        public static final double LEFT                         = 90.0;
        public static final double FRONT                        = 180.0;
        public static final double RIGHT                        = 270.0;
        public static final double TURTLE_POS                   = FRONT;
        public static final double TURTLE_DELAY                 = 0.0;

        // Preset positions.
        public static final double[] posPresets                 = new double[] {BACK, LEFT, FRONT, RIGHT};
        public static final double POS_PRESET_TOLERANCE         = 1.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.04, 0.0, 0.0, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor motor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Turret()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_TYPE, Params.MOTOR_INVERTED, true, true, Params.MOTOR_ID, null, null)
            .setLowerLimitSwitch(
                Params.LOWER_LIMIT_SWITCH_NAME, Params.LOWER_LIMIT_SWITCH_CHANNEL, Params.LOWER_LIMIT_SWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        motor = new FrcMotorActuator(motorParams).getMotor();
        motor.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(Params.posPidCoeffs)
                .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED), null);
        // Since we don't have upper limit switch, setting soft limits will protect turret from overrunning the upper
        // limit in manual mode.
        motor.setSoftPositionLimits(Params.MIN_POS, Params.MAX_POS, false);
    }   //Turret

    /**
     * This method returns the created motor.
     *
     * @return created motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

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

    private static final String DBKEY_POWER             = SUBSYSTEM_NAME + "/Power";        //String
    private static final String DBKEY_POSITION          = SUBSYSTEM_NAME + "/Position";     //String
    private static final String DBKEY_LOWER_LIMIT       = SUBSYSTEM_NAME + "/LowerLimit";   //Boolean

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_POWER, "");
        dashboard.refreshKey(DBKEY_POSITION, "");
        dashboard.refreshKey(DBKEY_LOWER_LIMIT, false);
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
            dashboard.putString(DBKEY_POWER, motor.getPower() + "/" + motor.getCurrent());
            dashboard.putString(DBKEY_POSITION, motor.getPosition() + "/" + motor.getPidTarget());
            dashboard.putBoolean(DBKEY_LOWER_LIMIT, motor.isLowerLimitSwitchActive());
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
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            double currValue = motor.getPosition();
            double paramValue = nextValueUp == null? currValue:
                motor.getNextPresetPosition(currValue, nextValueUp);

            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.posPidCoeffs)
                    .setPidControlParams(Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED)
                    .setTuningParams(paramValue));
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
        if (subsystemName.equalsIgnoreCase(Params.MOTOR_NAME))
        {
            TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
    
            motor.setPositionPidParameters(pidParams, null);
            motor.setPosition(pidParams.pidTarget);
            motor.tracer.traceInfo(instanceName, "Tune %s: PidParams=%s", subsystemName, pidParams);
        }
    }   //updateParamsFromDashboard

}   //class Turret
