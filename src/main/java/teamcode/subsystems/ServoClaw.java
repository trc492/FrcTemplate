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

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import frclib.driverio.FrcDashboard;
import frclib.subsystem.FrcServoClaw;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcServoClaw;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Claw Subsystem. This implementation consists of two servos and a sensor to detect the
 * presence of an object and can auto grab it. The sensor can be either a digital sensor such as touch sensor or beam
 * break sensor) or an analog sensor such as a distance sensor.
 */
public class ServoClaw extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "ServoClaw";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean USE_ANALOG_SENSOR           = true;
        public static final boolean USE_DIGITAL_SENSOR          = false;

        public static final String PRIMARY_SERVO_NAME           = SUBSYSTEM_NAME + ".leftClaw";
        public static final int PRIMARY_SERVO_CHANNEL           = 0;
        public static final boolean PRIMARY_SERVO_INVERTED      = false;

        public static final String FOLLOWER_SERVO_NAME          = SUBSYSTEM_NAME + ".rightClaw";
        public static final int FOLLOWER_SERVO_CHANNEL          = 1;
        public static final boolean FOLLOWER_SERVO_INVERTED     = !PRIMARY_SERVO_INVERTED;

        public static final double OPEN_POS                     = 0.2;
        public static final double OPEN_TIME                    = 0.5;
        public static final double CLOSE_POS                    = 0.55;
        public static final double CLOSE_TIME                   = 0.5;

        public static final String ANALOG_SENSOR_NAME           = SUBSYSTEM_NAME + ".sensor";
        public static final double LOWER_TRIGGER_THRESHOLD      = 2.0;
        public static final double UPPER_TRIGGER_THRESHOLD      = 3.0;
        public static final double TRIGGER_SETTLING_TIME        = 0.1;

        public static final String DIGITAL_SENSOR_NAME          = SUBSYSTEM_NAME + ".sensor";
        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;
    }   //class Params

    private static final String DBKEY_POSITION                  = Params.SUBSYSTEM_NAME + "/Position";
    private static final String DBKEY_IS_CLOSED                 = Params.SUBSYSTEM_NAME + "/IsClosed";
    private static final String DBKEY_HAS_OBJECT                = Params.SUBSYSTEM_NAME + "/HasObject";
    private static final String DBKEY_AUTO_ACTIVE               = Params.SUBSYSTEM_NAME + "/AutoActive";
    private static final String DBKEY_SENSOR_VALUE              = Params.SUBSYSTEM_NAME + "/SensorValue";
    private static final String DBKEY_SENSOR_STATE              = Params.SUBSYSTEM_NAME + "/SensorState";

    private final FrcDashboard dashboard;
    private final Rev2mDistanceSensor analogSensor;
    private final TrcServoClaw claw;

    /**
     * Constructor: Creates an instance of the object.
     */
    public ServoClaw()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_POSITION, 0.0);
        dashboard.refreshKey(DBKEY_IS_CLOSED, false);
        dashboard.refreshKey(DBKEY_HAS_OBJECT, false);
        dashboard.refreshKey(DBKEY_AUTO_ACTIVE, false);
        dashboard.refreshKey(DBKEY_SENSOR_VALUE, 0.0);
        dashboard.refreshKey(DBKEY_SENSOR_STATE, false);

        if (Params.USE_ANALOG_SENSOR)
        {
            analogSensor = new Rev2mDistanceSensor(Port.kOnboard);
            analogSensor.setAutomaticMode(true);
        }
        else
        {
            analogSensor = null;
        }

        FrcServoClaw.Params clawParams = new FrcServoClaw.Params()
            .setPrimaryServo(
                Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_CHANNEL, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(
                Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_CHANNEL, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME);

        if (analogSensor != null)
        {
            clawParams.setAnalogSourceTrigger(
                Params.ANALOG_SENSOR_NAME, this::getSensorData, Params.LOWER_TRIGGER_THRESHOLD,
                Params.UPPER_TRIGGER_THRESHOLD, Params.TRIGGER_SETTLING_TIME);
        }
        else if (Params.USE_DIGITAL_SENSOR)
        {
            clawParams.setDigitalInputTrigger(
                Params.DIGITAL_SENSOR_NAME, Params.SENSOR_DIGITAL_CHANNEL, Params.DIGITAL_TRIGGER_INVERTED);
        }

        claw = new FrcServoClaw(Params.SUBSYSTEM_NAME, clawParams).getClaw();
        claw.open();
    }   //ServoClaw

    /**
     * This method returns the created Servo Claw object.
     *
     * @return created claw.
     */
    public TrcServoClaw getClaw()
    {
        return claw;
    }   //getClaw

    /**
     * This method returns the current sensor value if it has one.
     *
     * @return sensor value if there is a sensor, 0.0 if there is none.
     */
    private double getSensorData()
    {
        if (analogSensor != null)
        {
            return analogSensor.isRangeValid()? analogSensor.getRange(): Double.MAX_VALUE;
        }
        else
        {
            return 0.0;
        }
    }   //getSensorData

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        claw.cancel();
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
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Don't move claw during turtle.
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
        dashboard.putNumber(DBKEY_POSITION, claw.getPosition());
        dashboard.putBoolean(DBKEY_IS_CLOSED, claw.isClosed());
        dashboard.putBoolean(DBKEY_HAS_OBJECT, claw.hasObject());
        dashboard.putBoolean(DBKEY_AUTO_ACTIVE, claw.isAutoActive());
        dashboard.putNumber(DBKEY_SENSOR_VALUE, claw.getSensorValue());
        dashboard.putBoolean(DBKEY_SENSOR_STATE, claw.getTriggerState());
        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param tuneParams specifies tuning parameters.
     */
    @Override
    public void prepSubsystemForTuning(double... tuneParams)
    {
        claw.setOpenClosePositions(tuneParams[0], tuneParams[1]);
    }   //prepSubsystemForTuning

}   //class ServoClaw
