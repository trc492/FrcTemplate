/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

import frclib.subsystem.FrcServoGrabber;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements an Elevator Subsystem.
 */
public class Grabber
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

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

        public static final boolean USE_REV_2M_SENSOR           = true;
        public static final double SENSOR_TRIGGER_THRESHOLD     = 2.0;
        public static final double HAS_OBJECT_THRESHOLD         = 2.0;
        public static final boolean ANALOG_TRIGGER_INVERTED     = true;

        public static final boolean USE_DIGITAL_SENSOR          = false;
        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean DIGITAL_TRIGGER_INVERTED    = false;
    }   //class Params

    private final Rev2mDistanceSensor rev2mSensor;
    private final TrcServoGrabber grabber;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Grabber()
    {
        if (Params.USE_REV_2M_SENSOR)
        {
            rev2mSensor = new Rev2mDistanceSensor(Port.kOnboard);
            rev2mSensor.setAutomaticMode(true);
        }
        else
        {
            rev2mSensor = null;
        }

        FrcServoGrabber.Params grabberParams = new FrcServoGrabber.Params()
            .setPrimaryServo(
                Params.PRIMARY_SERVO_NAME, Params.PRIMARY_SERVO_CHANNEL, Params.PRIMARY_SERVO_INVERTED)
            .setFollowerServo(
                Params.FOLLOWER_SERVO_NAME, Params.FOLLOWER_SERVO_CHANNEL, Params.FOLLOWER_SERVO_INVERTED)
            .setOpenCloseParams(Params.OPEN_POS, Params.OPEN_TIME, Params.CLOSE_POS, Params.CLOSE_TIME);

        if (rev2mSensor != null)
        {
            grabberParams.setAnalogSensorTrigger(
                this::getSensorData, Params.ANALOG_TRIGGER_INVERTED, Params.SENSOR_TRIGGER_THRESHOLD);
        }
        else if (Params.USE_DIGITAL_SENSOR)
        {
            grabberParams.setDigitalInputTrigger(Params.SENSOR_DIGITAL_CHANNEL, Params.DIGITAL_TRIGGER_INVERTED);
        }

        grabber = new FrcServoGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
        grabber.open();
    }   //Grabber

    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

    private double getSensorData()
    {
        if (rev2mSensor != null)
        {
            return rev2mSensor.getRange();
        }
        else
        {
            return 0.0;
        }
    }   //getSensorData

}   //class Grabber
