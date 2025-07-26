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
import frclib.subsystem.FrcIntake;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcIntake;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Elevator Subsystem.
 */
public class Intake extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Intake";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_TWO_MOTORS              = false;
        public static final boolean HAS_ENTRY_SENSOR            = true;
        public static final boolean HAS_EXIT_SENSOR             = false;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final int PRIMARY_MOTOR_ID                = 10;
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean PRIMARY_MOTOR_BRUSHLESS     = false;
        public static final boolean PRIMARY_MOTOR_ENC_ABS       = false;
        public static final boolean PRIMARY_MOTOR_INVERTED      = !HAS_TWO_MOTORS;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final int FOLLOWER_MOTOR_ID               = 12;
        public static final MotorType FOLLOWER_MOTOR_TYPE       = MotorType.CanTalonSrx;
        public static final boolean FOLLOWER_MOTOR_BRUSHLESS    = false;
        public static final boolean FOLLOWER_MOTOR_ENC_ABS      = false;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = PRIMARY_MOTOR_INVERTED;

        public static final int ENTRY_SENSOR_DIGITAL_CHANNEL    = 0;
        public static final boolean ENTRY_SENSOR_INVERTED       = false;

        public static final int EXIT_SENSOR_DIGITAL_CHANNEL     = 1;
        public static final boolean EXIT_SENSOR_INVERTED        = false;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class Params

    private static final String DBKEY_POWER                     = Params.SUBSYSTEM_NAME + "/Power";
    private static final String DBKEY_CURRENT                   = Params.SUBSYSTEM_NAME + "/Current";
    private static final String DBKEY_HAS_OBJECT                = Params.SUBSYSTEM_NAME + "/HasObject";
    private static final String DBKEY_ENTRY_SENSOR_STATE        = Params.SUBSYSTEM_NAME + "/EntrySensorState";
    private static final String DBKEY_EXIT_SENSOR_STATE         = Params.SUBSYSTEM_NAME + "/EntrySensorState";

    private final FrcDashboard dashboard;
    private final TrcIntake intake;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_POWER, 0.0);
        dashboard.refreshKey(DBKEY_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_HAS_OBJECT, false);
        dashboard.refreshKey(DBKEY_ENTRY_SENSOR_STATE, false);
        dashboard.refreshKey(DBKEY_EXIT_SENSOR_STATE, false);

        FrcIntake.Params intakeParams = new FrcIntake.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_ID, Params.PRIMARY_MOTOR_TYPE,
                Params.PRIMARY_MOTOR_BRUSHLESS, Params.PRIMARY_MOTOR_ENC_ABS, Params.PRIMARY_MOTOR_INVERTED);
        if (Params.HAS_TWO_MOTORS)
        {
            intakeParams.setFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_ID, Params.FOLLOWER_MOTOR_TYPE,
                Params.FOLLOWER_MOTOR_BRUSHLESS, Params.FOLLOWER_MOTOR_ENC_ABS, Params.FOLLOWER_MOTOR_INVERTED);
        }

        if (Params.HAS_ENTRY_SENSOR)
        {
            intakeParams.setEntryDigitalInput(
                Params.ENTRY_SENSOR_DIGITAL_CHANNEL, Params.ENTRY_SENSOR_INVERTED, null);
        }

        if (Params.HAS_EXIT_SENSOR)
        {
            intakeParams.setExitDigitalInput(
                Params.EXIT_SENSOR_DIGITAL_CHANNEL, Params.EXIT_SENSOR_INVERTED, null);
        }
        intake = new FrcIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();
    }   //Intake

    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        intake.cancel();
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
        // Intake does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Intake does not support resetState.
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
        dashboard.putNumber(DBKEY_POWER, intake.getPower());
        dashboard.putNumber(DBKEY_CURRENT, intake.getCurrent());
        dashboard.putBoolean(DBKEY_HAS_OBJECT, intake.hasObject());
        dashboard.putBoolean(DBKEY_ENTRY_SENSOR_STATE, intake.getSensorState(intake.entryTrigger));
        dashboard.putBoolean(DBKEY_EXIT_SENSOR_STATE, intake.getSensorState(intake.exitTrigger));
        return lineNum;
    }   //updateStatus

}   //class Intake
