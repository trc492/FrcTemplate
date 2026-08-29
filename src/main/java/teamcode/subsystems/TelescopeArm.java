/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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
import trclib.timer.TrcTimer;

/**
 * This class implements an Arm Subsystem. This implementation consists of a motor with built-in encoder. It does
 * not have any limit switches, so it is using motor stall detection to zero calibrate the built-in relative encoder.
 * It supports gravity compensation by computing the power required to hold the arm at its current angle.
 */
public class TelescopeArm extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME                   = "TelescopeArm";
    public static final boolean NEED_ZERO_CAL                   = true;
    public static final double GOBILDA312_CPR                   = (((1.0 + (46.0/17.0))) * (1.0 + (46.0/11.0))) * 28.0;

    public static final class Params
    {
        public static final boolean TELESCOPE_HAS_LOWER_LIMIT_SW= false;
        public static final boolean TELESCOPE_HAS_UPPER_LIMIT_SW= false;

        public static final boolean HAS_TILTER                  = false;
        public static final boolean TILTER_HAS_LOWER_LIMIT_SW   = false;
        public static final boolean TILTER_HAS_UPPER_LIMIT_SW   = false;

        // Telescope Parameters.
        public static final MotorType TELESCOPE_MOTOR_TYPE      = MotorType.CanTalonSrx;
        public static final String TELESCOPE_MOTOR_NAME         = SUBSYSTEM_NAME + ".telescopeMotor";
        public static final int TELESCOPE_MOTOR_ID              = 10;
        public static final boolean TELESCOPE_MOTOR_INVERTED    = false;

        public static final String TELESCOPE_LOWER_LIMIT_SW_NAME= SUBSYSTEM_NAME + ".telescopeLowerLimit";
        public static final int TELESCOPE_LOWER_LIMIT_SW_CHANNEL= 0;
        public static final boolean TELESCOPE_LOWER_LIMIT_SW_INVERTED = false;

        public static final String TELESCOPE_UPPER_LIMIT_SW_NAME= SUBSYSTEM_NAME + ".telescopeUpperLimit";
        public static final int TELESCOPE_UPPER_LIMIT_SW_CHANNEL= 1;
        public static final boolean TELESCOPE_UPPER_LIMIT_SW_INVERTED = false;

        public static final double TELESCOPE_POS_OFFSET         = 17.5;
        public static final double TELESCOPE_INCHES_PER_COUNT   = (37.75 - TELESCOPE_POS_OFFSET)/2323.0;
        public static final double TELESCOPE_POWER_LIMIT        = 1.0;
        public static final double TELESCOPE_ZERO_CAL_POWER     = -0.25;
        public static final double TELESCOPE_ZERO_CAL_TIMEOUT   = 0.0;

        public static final double TELESCOPE_MIN_POS            = TELESCOPE_POS_OFFSET;
        public static final double TELESCOPE_MAX_POS            = 37.0;
        public static final double TELESCOPE_TURTLE_POS         = TELESCOPE_MIN_POS;
        public static final double TELESCOPE_TURTLE_DELAY       = 0.0;
        public static final double[] telescopePosPresets        = 
            {TELESCOPE_MIN_POS, 20.0, 25.0, 30.0, 35.0, TELESCOPE_MAX_POS};
        public static final double TELESCOPE_POS_PRESET_TOLERANCE = 1.0;

        public static final boolean TELESCOPE_USE_SOFTWARE_PID  = true;
        public static final TrcPidController.PidCoefficients telescopePosPidCoeffs =
            new TrcPidController.PidCoefficients(0.5, 0.0, 0.0, 0.0, 0.0);
        public static final double TELESCOPE_POS_PID_TOLERANCE  = 0.5;
        public static final double TELESCOPE_GRAVITY_COMP_POWER = 0.0;
        // Since we don't have lower limit switch, must enable Stall Protection to do zero calibration by stalling.
        public static final double TELESCOPE_STALL_MIN_POWER    = Math.abs(TELESCOPE_ZERO_CAL_POWER);
        public static final double TELESCOPE_STALL_TOLERANCE    = 0.1;
        public static final double TELESCOPE_STALL_TIMEOUT      = 0.1;
        public static final double TELESCOPE_STALL_RESET_TIMEOUT= 0.0;

        // Tilter Parameters.
        public static final MotorType TILTER_MOTOR_TYPE         = MotorType.CanTalonSrx;
        public static final String TILTER_MOTOR_NAME            = SUBSYSTEM_NAME + ".tilterMotor";
        public static final int TILTER_MOTOR_ID                 = 12;
        public static final boolean TILTER_MOTOR_INVERTED       = true;

        public static final String TILTER_LOWER_LIMIT_SW_NAME   = SUBSYSTEM_NAME + ".tilterLowerLimit";
        public static final int TILTER_LOWER_LIMIT_SW_CHANNEL   = 2;
        public static final boolean TILTER_LOWER_LIMIT_SW_INVERTED = false;

        public static final String TILTER_UPPER_LIMIT_SW_NAME   = SUBSYSTEM_NAME + ".tilterUpperLimit";
        public static final int TILTER_UPPER_LIMIT_SW_CHANNEL   = 3;
        public static final boolean TILTER_UPPER_LIMIT_SW_INVERTED = false;

        public static final double TILTER_DEG_PER_COUNT         = 360.0 / GOBILDA312_CPR;
        public static final double TILTER_POS_OFFSET            = 0.0;
        public static final double TILTER_POWER_LIMIT           = 0.25;
        public static final double TILTER_ZERO_CAL_POWER        = -0.2;
        public static final double TILTER_ZERO_CAL_TIMEOUT      = 0.0;

        public static final double TILTER_MIN_POS               = TILTER_POS_OFFSET;
        public static final double TILTER_MAX_POS               = 90.0;
        public static final double TILTER_TURTLE_POS            = TILTER_MIN_POS;
        public static final double TILTER_TURTLE_DELAY          = 0.0;
        public static final double[] tilterPosPresets           = 
            {TILTER_MIN_POS, 15.0, 30.0, 45.0, 60.0, 75.0, TILTER_MAX_POS};
        public static final double TILTER_POS_PRESET_TOLERANCE  = 5.0;

        public static final boolean TILTER_USE_SOFTWARE_PID     = true;
        public static final TrcPidController.PidCoefficients tilterPosPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.001, 0.0, 0.0);
        public static final double TILTER_POS_PID_TOLERANCE     = 1.0;
        public static final double TILTER_GRAVITY_COMP_POWER    = 0.161;
        // Since we don't have lower limit switch, must enable Stall Protection to do zero calibration by stalling.
        public static final double TILTER_STALL_MIN_POWER       = Math.abs(TILTER_ZERO_CAL_POWER);
        public static final double TILTER_STALL_TOLERANCE       = 0.1;
        public static final double TILTER_STALL_TIMEOUT         = 0.1;
        public static final double TILTER_STALL_RESET_TIMEOUT   = 0.0;
    }   //class Params

    private final FrcDashboard dashboard;
    public final TrcMotor telescope;
    public final TrcMotor tilter;
    private final TrcTimer timer;
    private Double tuneTelescopeGravityCompPower = null;
    private Double tuneTilterGravityCompPower = null;
    private String tuneSubsystemName = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public TelescopeArm()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);
        dashboard = FrcDashboard.getInstance();

        // Create Telescope.
        FrcMotorActuator.Params telescopeMotorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.TELESCOPE_MOTOR_NAME, Params.TELESCOPE_MOTOR_TYPE, Params.TELESCOPE_MOTOR_INVERTED, true, true,
                Params.TELESCOPE_MOTOR_ID, null, null)
            .setPositionScaleAndOffset(Params.TELESCOPE_INCHES_PER_COUNT, Params.TELESCOPE_POS_OFFSET)
            .setPositionPresets(Params.TELESCOPE_POS_PRESET_TOLERANCE, Params.telescopePosPresets);

        if (Params.TELESCOPE_HAS_LOWER_LIMIT_SW)
        {
            telescopeMotorParams.setLowerLimitSwitch(
                Params.TELESCOPE_LOWER_LIMIT_SW_NAME, Params.TELESCOPE_LOWER_LIMIT_SW_CHANNEL,
                Params.TELESCOPE_LOWER_LIMIT_SW_INVERTED);
        }

        if (Params.TELESCOPE_HAS_UPPER_LIMIT_SW)
        {
            telescopeMotorParams.setUpperLimitSwitch(
                Params.TELESCOPE_UPPER_LIMIT_SW_NAME, Params.TELESCOPE_UPPER_LIMIT_SW_CHANNEL,
                Params.TELESCOPE_UPPER_LIMIT_SW_INVERTED);
        }

        telescope = new FrcMotorActuator(telescopeMotorParams).getMotor();
        telescope.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(Params.telescopePosPidCoeffs)
                .setPidControlParams(Params.TELESCOPE_POS_PID_TOLERANCE, Params.TELESCOPE_USE_SOFTWARE_PID), null);
        telescope.setPositionPidPowerComp(this::getTelescopeGravityComp);
        telescope.setStallProtection(
            Params.TELESCOPE_STALL_MIN_POWER, Params.TELESCOPE_STALL_TOLERANCE, Params.TELESCOPE_STALL_TIMEOUT,
            Params.TELESCOPE_STALL_RESET_TIMEOUT);
        telescope.setSoftPositionLimits(Params.TELESCOPE_MIN_POS, Params.TELESCOPE_MAX_POS, false);

        // Create Tilter.
        if (Params.HAS_TILTER)
        {
            FrcMotorActuator.Params tilterMotorParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.TILTER_MOTOR_NAME, Params.TILTER_MOTOR_TYPE, Params.TILTER_MOTOR_INVERTED, true, true,
                    Params.TILTER_MOTOR_ID, null, null)
                .setPositionScaleAndOffset(Params.TILTER_DEG_PER_COUNT, Params.TILTER_POS_OFFSET)
                .setPositionPresets(Params.TILTER_POS_PRESET_TOLERANCE, Params.tilterPosPresets);

            if (Params.TILTER_HAS_LOWER_LIMIT_SW)
            {
                tilterMotorParams.setLowerLimitSwitch(
                    Params.TILTER_LOWER_LIMIT_SW_NAME, Params.TILTER_LOWER_LIMIT_SW_CHANNEL,
                    Params.TILTER_LOWER_LIMIT_SW_INVERTED);
            }

            if (Params.TILTER_HAS_UPPER_LIMIT_SW)
            {
                tilterMotorParams.setUpperLimitSwitch(
                    Params.TILTER_UPPER_LIMIT_SW_NAME, Params.TILTER_UPPER_LIMIT_SW_CHANNEL,
                    Params.TILTER_UPPER_LIMIT_SW_INVERTED);
            }

            tilter = new FrcMotorActuator(tilterMotorParams).getMotor();
            tilter.setPositionPidParameters(
                new PidParams()
                    .setPidCoefficients(Params.tilterPosPidCoeffs)
                    .setPidControlParams(Params.TILTER_POS_PID_TOLERANCE, Params.TILTER_USE_SOFTWARE_PID), null);
            tilter.setPositionPidPowerComp(this::getTilterGravityComp);
            tilter.setStallProtection(
                Params.TILTER_STALL_MIN_POWER, Params.TILTER_STALL_TOLERANCE, Params.TILTER_STALL_TIMEOUT,
                Params.TILTER_STALL_RESET_TIMEOUT);
            tilter.setSoftPositionLimits(Params.TILTER_MIN_POS, Params.TILTER_MAX_POS, false);
        }
        else
        {
            tilter = null;
        }

        timer = new TrcTimer(SUBSYSTEM_NAME + ".timer");
    }   //MotorArm

    /**
     * This method calculates the power required to make the telescope gravity neutral.
     *
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getTelescopeGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneTelescopeGravityCompPower != null?
            tuneTelescopeGravityCompPower: Params.TELESCOPE_GRAVITY_COMP_POWER;

        if (tilter != null)
        {
            // There is a tilter, compensation should take into account of tilt angle.
            gravityCompPower *= Math.cos(Math.toRadians(tilter.getPosition()));

        }

        return gravityCompPower;
    }   //getTelescopeGravityComp

    /**
     * This method calculates the power required to make the tilter gravity neutral.
     *
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getTilterGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneTilterGravityCompPower != null?
            tuneTilterGravityCompPower: Params.TILTER_GRAVITY_COMP_POWER;

        gravityCompPower *= Math.cos(Math.toRadians(tilter.getPosition()));

        return gravityCompPower;
    }   //getTilterGravityComp

    /**
     * This method sets the telescope arm position including extending the telescope and tilting the arm.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param telescopePos specifies the telescope extend position.
     * @param tilterPos specifies the tilter angle.
     * @param completionEvent specifies an event to signal when completion, can be null if not provided.
     * @param timeout specifies timeout in seconds, can be zero if no timeout.
     */
    public void setPosition(
        String owner, Double telescopePos, Double tilterPos, TrcEvent completionEvent, double timeout)
    {
        TrcEvent telescopeEvent = null;
        TrcEvent tilterEvent = null;
        TrcEvent timeoutEvent = null;

        if (telescopePos != null)
        {
            telescopeEvent = new TrcEvent(SUBSYSTEM_NAME + ".telescopeEvent");
            telescope.setPosition(owner, 0.0, telescopePos, true, Params.TELESCOPE_POWER_LIMIT, telescopeEvent, 0.0);
        }

        if (tilter != null && tilterPos != null)
        {
            tilterEvent = new TrcEvent(SUBSYSTEM_NAME + ".tilterEvent");
            tilter.setPosition(owner, 0.0, tilterPos, true, Params.TILTER_POWER_LIMIT, tilterEvent, 0.0);
        }

        if (timeout > 0.0 && (telescopeEvent != null || tilterEvent != null))
        {
            timeoutEvent = new TrcEvent(SUBSYSTEM_NAME + ".timeoutEvent");
            timeoutEvent.setCallback(this::handleTimeout, null);
            timer.set(TrcTimer.getCurrentTime() + timeout, timeoutEvent);
        }

        if (completionEvent != null && (telescopeEvent != null || tilterEvent != null))
        {
            completionEvent.signalOnEvents(true, false, telescopeEvent, tilterEvent);
        }
    }   //setPosition

    /**
     * This method is called when the setPosition operation is timed out.
     *
     * @param context not used.
     * @param canceled not used.
     */
    private void handleTimeout(Object context, boolean canceled)
    {
        // By canceling both the telescope and the tilter, it will cause telescopeEvent and tilterEvent to be canceled
        // as well. That in turn will cause completionEvent to be canceled.
        telescope.cancel();
        if (tilter != null)
        {
            tilter.cancel();
        }
    }   //handleTimeout

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        telescope.cancel();
        if (tilter != null)
        {
            tilter.cancel();
        }
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
        TrcEvent telescopeEvent = new TrcEvent(SUBSYSTEM_NAME + ".telescopeEvent");
        TrcEvent tilterEvent = null;

        telescope.zeroCalibrate(
            owner, Params.TELESCOPE_ZERO_CAL_POWER, telescopeEvent, Params.TELESCOPE_ZERO_CAL_TIMEOUT);

        if (tilter != null)
        {
            tilterEvent = new TrcEvent(SUBSYSTEM_NAME + ".tilterEvent");
            tilter.zeroCalibrate(owner, Params.TILTER_ZERO_CAL_POWER, tilterEvent, Params.TILTER_ZERO_CAL_TIMEOUT);
        }

        if (completionEvent != null)
        {
            completionEvent.signalOnEvents(true, false, telescopeEvent, tilterEvent);
        }
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        telescope.setPosition(
            Params.TELESCOPE_TURTLE_DELAY, Params.TELESCOPE_TURTLE_POS, true, Params.TELESCOPE_POWER_LIMIT);

        if (tilter != null)
        {
            tilter.setPosition(
                Params.TILTER_TURTLE_DELAY, Params.TILTER_TURTLE_POS, true, Params.TILTER_POWER_LIMIT);
        }
    }   //resetState

    private static final String DBKEY_TELESCOPE_PWR_INFO    = SUBSYSTEM_NAME + "/TelescopePwrInfo";     //String
    private static final String DBKEY_TELESCOPE_POS_INFO    = SUBSYSTEM_NAME + "/TelescopePosInfo";     //String
    private static final String DBKEY_TELESCOPE_LOWER_LIMIT = SUBSYSTEM_NAME + "/TelescopeLowerLimit";  //Boolean
    private static final String DBKEY_TELESCOPE_UPPER_LIMIT = SUBSYSTEM_NAME + "/TelescopeUpperLimit";  //Boolean
    private static final String DBKEY_TILTER_PWR_INFO       = SUBSYSTEM_NAME + "/TilterPwrInfo";        //String
    private static final String DBKEY_TILTER_POS_INFO       = SUBSYSTEM_NAME + "/TilterPosInfo";        //String
    private static final String DBKEY_TILTER_LOWER_LIMIT    = SUBSYSTEM_NAME + "/TilterLowerLimit";     //Boolean
    private static final String DBKEY_TILTER_UPPER_LIMIT    = SUBSYSTEM_NAME + "/TilterUpperLimit";     //Boolean

    /**
     * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
     */
    @Override
    public void publishToDashboard()
    {
        dashboard.refreshKey(DBKEY_TELESCOPE_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_TELESCOPE_POS_INFO, "");
        dashboard.refreshKey(DBKEY_TELESCOPE_LOWER_LIMIT, false);
        dashboard.refreshKey(DBKEY_TELESCOPE_UPPER_LIMIT, false);
        dashboard.refreshKey(DBKEY_TILTER_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_TILTER_POS_INFO, "");
        dashboard.refreshKey(DBKEY_TILTER_LOWER_LIMIT, false);
        dashboard.refreshKey(DBKEY_TILTER_UPPER_LIMIT, false);
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
            dashboard.putString(DBKEY_TELESCOPE_PWR_INFO, telescope.getPower() + "/" + telescope.getCurrent());
            dashboard.putString(DBKEY_TELESCOPE_POS_INFO, telescope.getPosition() + "/" + telescope.getPidTarget());

            if (Params.TELESCOPE_HAS_LOWER_LIMIT_SW)
            {
                dashboard.putBoolean(DBKEY_TELESCOPE_LOWER_LIMIT, telescope.isLowerLimitSwitchActive());
            }

            if (Params.TELESCOPE_HAS_UPPER_LIMIT_SW)
            {
                dashboard.putBoolean(DBKEY_TELESCOPE_UPPER_LIMIT, telescope.isUpperLimitSwitchActive());
            }

            if (tilter != null)
            {
                dashboard.putString(DBKEY_TILTER_PWR_INFO, tilter.getPower() + "/" + tilter.getCurrent());
                dashboard.putString(DBKEY_TILTER_POS_INFO, tilter.getPosition() + "/" + tilter.getPidTarget());

                if (Params.TILTER_HAS_LOWER_LIMIT_SW)
                {
                    dashboard.putBoolean(DBKEY_TILTER_LOWER_LIMIT, tilter.isLowerLimitSwitchActive());
                }

                if (Params.TILTER_HAS_UPPER_LIMIT_SW)
                {
                    dashboard.putBoolean(DBKEY_TILTER_UPPER_LIMIT, tilter.isUpperLimitSwitchActive());
                }
            }
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null)
        {
            if (tuneSubsystemName.equalsIgnoreCase(Params.TELESCOPE_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, telescope.getPosition());
            }
            else if (tilter != null && tuneSubsystemName.equalsIgnoreCase(Params.TILTER_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, tilter.getPosition());
            }
        }

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
        if (subsystemName.equalsIgnoreCase(Params.TELESCOPE_MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.telescopePosPidCoeffs)
                    .setPidControlParams(Params.TELESCOPE_POS_PID_TOLERANCE, Params.TELESCOPE_USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.TELESCOPE_MIN_POS);
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.TELESCOPE_GRAVITY_COMP_POWER);
        }
        else if (tilter != null && subsystemName.equalsIgnoreCase(Params.TILTER_MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(Params.tilterPosPidCoeffs)
                    .setPidControlParams(Params.TILTER_POS_PID_TOLERANCE, Params.TILTER_USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.TILTER_MIN_POS);
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.TILTER_GRAVITY_COMP_POWER);
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
        TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
        Double target = null;
        double gravityPower = 0.0;

        tuneSubsystemName = null;
        if (subsystemName.equalsIgnoreCase(Params.TELESCOPE_MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.TELESCOPE_MIN_POS);
            tuneTelescopeGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.TELESCOPE_GRAVITY_COMP_POWER);
            telescope.setPositionPidParameters(pidParams, null);
            telescope.setPosition(target);
            gravityPower = tuneTelescopeGravityCompPower;
            tuneSubsystemName = subsystemName;
        }
        else if (tilter != null && subsystemName.equalsIgnoreCase(Params.TILTER_MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, Params.TILTER_MIN_POS);
            tuneTilterGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, Params.TILTER_GRAVITY_COMP_POWER);
            tilter.setPositionPidParameters(pidParams, null);
            tilter.setPosition(target);
            gravityPower = tuneTilterGravityCompPower;
            tuneSubsystemName = subsystemName;
        }

        if (target != null)
        {
            // Matched a subsystem name.
            telescope.tracer.traceInfo(
                instanceName, "Tune %s: PidParams=%s, target=%.3f, GravityPower=%.3f",
                subsystemName, pidParams, target, gravityPower);
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
        Double target = null;

        if (subsystemName.equalsIgnoreCase(Params.TELESCOPE_MOTOR_NAME))
        {
            target = telescope.presetPositionUp(null, null);
        }
        else if (tilter != null && subsystemName.equalsIgnoreCase(Params.TILTER_MOTOR_NAME))
        {
            target = tilter.presetPositionUp(null, null);
        }

        if (target != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, target);
            telescope.tracer.traceInfo(instanceName, "Tune %s Up: target=%.3f", subsystemName, target);
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
        Double target = null;

        if (subsystemName.equalsIgnoreCase(Params.TELESCOPE_MOTOR_NAME))
        {
            target = telescope.presetPositionDown(null, null);
        }
        else if (tilter != null && subsystemName.equalsIgnoreCase(Params.TILTER_MOTOR_NAME))
        {
            target = tilter.presetPositionDown(null, null);
        }

        if (target != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, target);
            telescope.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class TelescopeArm
