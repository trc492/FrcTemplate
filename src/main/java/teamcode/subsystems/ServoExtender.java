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
import teamcode.FrcTest;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the Servo Extender subsystem. This implementation is a linear extender driven by two servos
 * to either extend or retract the extender.
 */
public class ServoExtender extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME                   = "ServoExtender";
    public static final boolean NEED_ZERO_CAL                   = false;

    public static class Params
    {
        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final int PRIMARY_SERVO_CHANNEL           = 0;
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final int FOLLOWER_SERVO_CHANNEL          = 1;
        public static final boolean FOLLOWER_SERVO_INVERTED     = false;

        public static double POS_RETRACT                        = 0.1;
        public static double POS_EXTEND                         = 0.8;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcServo servo;
    private double tuneLogicalPos = Params.POS_RETRACT;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoExtender()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        FrcServoActuator.Params extenderParams = new FrcServoActuator.Params()
            .setPrimaryServo(Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_CHANNEL, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_CHANNEL, Params.FOLLOWER_SERVO_INVERTED);

        servo = new FrcServoActuator(extenderParams).getServo();
    }   //ServoExtender

    /**
     * This method returns the created servo.
     *
     * @return created servo.
     */
    public TrcServo getServo()
    {
        return servo;
    }   //getServo

    /**
     * This method checks if the extender is extended.
     *
     * @return true if extended, false otherwise.
     */
    public boolean isExtended()
    {
        return servo.getPosition() == Params.POS_EXTEND;
    }   //isExtended

    /**
     * This method sets the extender to extended position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(owner, delay, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, delay, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     *
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void extend(TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, 0.0, Params.POS_EXTEND, completionEvent, timeout);
    }   //extend

    /**
     * This method sets the extender to extended position.
     */
    public void extend()
    {
        servo.setPosition(null, 0.0, Params.POS_EXTEND, null, 0.0);
    }   //extend

    /**
     * This method sets the extender to retracted position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(String owner, double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(owner, delay, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     *
     * @param delay specifies the delay in seconds before setting the position of the servo, can be zero if no delay.
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(double delay, TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, delay, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     *
     * @param completionEvent specifies an event object to signal when the timeout event has expired.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     */
    public void retract(TrcEvent completionEvent, double timeout)
    {
        servo.setPosition(null, 0.0, Params.POS_RETRACT, completionEvent, timeout);
    }   //retract

    /**
     * This method sets the extender to retracted position.
     */
    public void retract()
    {
        servo.setPosition(null, 0.0, Params.POS_RETRACT, null, 0.0);
    }   //retract

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        servo.cancel();
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
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        servo.setPosition(Params.POS_RETRACT);
    }   //resetState

    private static final String DBKEY_POS               = SUBSYSTEM_NAME + "/Pos";          //Number
    private static final String DBKEY_IS_EXTENDED       = SUBSYSTEM_NAME + "/IsExtended";   //Boolean

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_POS, 0.0);
        dashboard.refreshKey(DBKEY_IS_EXTENDED, false);
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
            dashboard.putNumber(DBKEY_POS, servo.getPosition());
            dashboard.putBoolean(DBKEY_IS_EXTENDED, isExtended());
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            double currValue = servo.getPosition();

            tuneLogicalPos = nextValueUp == null? currValue:
                             nextValueUp? Params.POS_EXTEND: Params.POS_RETRACT;
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams().setTuningParams(tuneLogicalPos));
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
        if (subsystemName.equalsIgnoreCase(Params.PRIMARY_SERVO_NAME))
        {
            servo.setPosition(tuneLogicalPos);
            servo.tracer.traceInfo(instanceName, "Tune %s: LogicalPos=%f", subsystemName, tuneLogicalPos);
        }
    }   //updateParamsFromDashboard

}   //class ServoExtender
