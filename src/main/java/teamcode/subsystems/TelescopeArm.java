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
    public static final String SUBSYSTEM_NAME = "TelescopeArm";
    private static final boolean NEED_ZERO_CAL = true;
    private static final double GOBILDA312_CPR = (((1.0 + (46.0/17.0))) * (1.0 + (46.0/11.0))) * 28.0;

    private static final boolean HAS_ELBOW = false;

    public static final class TelescopeParams
    {
        private static final boolean HAS_LOWER_LIMIT_SW         = false;
        private static final boolean HAS_UPPER_LIMIT_SW         = false;

        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".telescopeMotor";
        public static final int MOTOR_ID                        = 10;
        public static final boolean MOTOR_INVERTED              = false;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final String LOWER_LIMIT_SW_NAME          = SUBSYSTEM_NAME + ".telescopeLowerLimit";
        public static final int LOWER_LIMIT_SW_CHANNEL          = 0;
        public static final boolean LOWER_LIMIT_SW_INVERTED     = false;

        public static final String UPPER_LIMIT_SW_NAME          = SUBSYSTEM_NAME + ".telescopeUpperLimit";
        public static final int UPPER_LIMIT_SW_CHANNEL          = 1;
        public static final boolean UPPER_LIMIT_SW_INVERTED     = false;

        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.5, 0.0, 0.0, 0.0, 0.0);

        public static final double POS_OFFSET                   = 17.5;
        public static final double INCHES_PER_COUNT             = (37.75 - POS_OFFSET)/2323.0;
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 37.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double POS_PRESET_TOLERANCE         = 1.0;
        public static final double[] posPresets                 = {MIN_POS, 20.0, 25.0, 30.0, 35.0, MAX_POS};

        public static final double POWER_LIMIT                  = 1.0;
        public static final double GRAVITY_COMP_POWER           = 0.0;
        public static final double ZERO_CAL_POWER               = -0.25;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;

        // Since we don't have lower limit switch, must enable Stall Protection to do zero calibration by stalling.
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class TelescopeParams

    public static class ElbowParams
    {
        private static final boolean HAS_LOWER_LIMIT_SW         = false;
        private static final boolean HAS_UPPER_LIMIT_SW         = false;

        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".elbowMotor";
        public static final int MOTOR_ID                        = 12;
        public static final boolean MOTOR_INVERTED              = true;
        public static final boolean MOTOR_VOLTCOMP_ENABLED      = true;
        public static final boolean MOTOR_BRAKE_ENABLED         = true;

        public static final String LOWER_LIMIT_SW_NAME          = SUBSYSTEM_NAME + ".elbowLowerLimit";
        public static final int LOWER_LIMIT_SW_CHANNEL          = 2;
        public static final boolean LOWER_LIMIT_SW_INVERTED     = false;

        public static final String UPPER_LIMIT_SW_NAME          = SUBSYSTEM_NAME + ".elbowUpperLimit";
        public static final int UPPER_LIMIT_SW_CHANNEL          = 3;
        public static final boolean UPPER_LIMIT_SW_INVERTED     = false;

        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final boolean USE_SOFTWARE_PID            = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.0, 0.001, 0.0, 0.0);

        public static final double DEG_PER_COUNT                = 360.0 / GOBILDA312_CPR;
        public static final double POS_OFFSET                   = 0.0;
        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 90.0;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double POS_PRESET_TOLERANCE         = 5.0;
        public static final double[] posPresets                 = {MIN_POS, 15.0, 30.0, 45.0, 60.0, 75.0, MAX_POS};

        public static final double POWER_LIMIT                  = 0.25;
        public static final double GRAVITY_COMP_POWER           = 0.161;
        public static final double ZERO_CAL_POWER               = -0.2;
        public static final double ZERO_CAL_TIMEOUT             = 0.0;

        // Since we don't have lower limit switch, must enable Stall Protection to do zero calibration by stalling.
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class ElbowParams

    private final FrcDashboard dashboard;
    public final TrcMotor telescope;
    public final TrcMotor elbow;
    private final TrcTimer timer;
    private String tuneSubsystemName = null;
    private Double tuneTelescopeGravityCompPower = null;
    private Double tuneElbowGravityCompPower = null;

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
                TelescopeParams.MOTOR_NAME, TelescopeParams.MOTOR_TYPE, TelescopeParams.MOTOR_INVERTED,
                TelescopeParams.MOTOR_VOLTCOMP_ENABLED, TelescopeParams.MOTOR_BRAKE_ENABLED,
                TelescopeParams.MOTOR_ID, null, null)
            .setPositionScaleAndOffset(TelescopeParams.INCHES_PER_COUNT, TelescopeParams.POS_OFFSET)
            .setPositionPresets(TelescopeParams.POS_PRESET_TOLERANCE, TelescopeParams.posPresets);

        if (TelescopeParams.HAS_LOWER_LIMIT_SW)
        {
            telescopeMotorParams.setLowerLimitSwitch(
                TelescopeParams.LOWER_LIMIT_SW_NAME, TelescopeParams.LOWER_LIMIT_SW_CHANNEL,
                TelescopeParams.LOWER_LIMIT_SW_INVERTED);
        }

        if (TelescopeParams.HAS_UPPER_LIMIT_SW)
        {
            telescopeMotorParams.setUpperLimitSwitch(
                TelescopeParams.UPPER_LIMIT_SW_NAME, TelescopeParams.UPPER_LIMIT_SW_CHANNEL,
                TelescopeParams.UPPER_LIMIT_SW_INVERTED);
        }

        telescope = new FrcMotorActuator(telescopeMotorParams).getMotor();
        telescope.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(TelescopeParams.posPidCoeffs)
                .setPidControlParams(TelescopeParams.POS_PID_TOLERANCE, TelescopeParams.USE_SOFTWARE_PID), null);
        telescope.setPositionPidPowerComp(this::getTelescopeGravityComp);
        telescope.setStallProtection(
            TelescopeParams.STALL_MIN_POWER, TelescopeParams.STALL_TOLERANCE, TelescopeParams.STALL_TIMEOUT,
            TelescopeParams.STALL_RESET_TIMEOUT);
        telescope.setSoftPositionLimits(TelescopeParams.MIN_POS, TelescopeParams.MAX_POS, false);

        // Create Elbow.
        if (HAS_ELBOW)
        {
            FrcMotorActuator.Params elbowMotorParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    ElbowParams.MOTOR_NAME, ElbowParams.MOTOR_TYPE, ElbowParams.MOTOR_INVERTED,
                    ElbowParams.MOTOR_VOLTCOMP_ENABLED, ElbowParams.MOTOR_BRAKE_ENABLED,
                    ElbowParams.MOTOR_ID, null, null)
                .setPositionScaleAndOffset(ElbowParams.DEG_PER_COUNT, ElbowParams.POS_OFFSET)
                .setPositionPresets(ElbowParams.POS_PRESET_TOLERANCE, ElbowParams.posPresets);

            if (ElbowParams.HAS_LOWER_LIMIT_SW)
            {
                elbowMotorParams.setLowerLimitSwitch(
                    ElbowParams.LOWER_LIMIT_SW_NAME, ElbowParams.LOWER_LIMIT_SW_CHANNEL,
                    ElbowParams.LOWER_LIMIT_SW_INVERTED);
            }

            if (ElbowParams.HAS_UPPER_LIMIT_SW)
            {
                elbowMotorParams.setUpperLimitSwitch(
                    ElbowParams.UPPER_LIMIT_SW_NAME, ElbowParams.UPPER_LIMIT_SW_CHANNEL,
                    ElbowParams.UPPER_LIMIT_SW_INVERTED);
            }

            elbow = new FrcMotorActuator(elbowMotorParams).getMotor();
            elbow.setPositionPidParameters(
                new PidParams()
                    .setPidCoefficients(ElbowParams.posPidCoeffs)
                    .setPidControlParams(ElbowParams.POS_PID_TOLERANCE, ElbowParams.USE_SOFTWARE_PID), null);
            elbow.setPositionPidPowerComp(this::getElbowGravityComp);
            elbow.setStallProtection(
                ElbowParams.STALL_MIN_POWER, ElbowParams.STALL_TOLERANCE, ElbowParams.STALL_TIMEOUT,
                ElbowParams.STALL_RESET_TIMEOUT);
            elbow.setSoftPositionLimits(ElbowParams.MIN_POS, ElbowParams.MAX_POS, false);
        }
        else
        {
            elbow = null;
        }

        timer = new TrcTimer(SUBSYSTEM_NAME + ".timer");
    }   //MotorArm

    /**
     * This method calculates the power required to make the telescope gravity neutral.
     *
     * @param motor specifies the motor for determining its gravity comp power.
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getTelescopeGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneTelescopeGravityCompPower != null?
            tuneTelescopeGravityCompPower: TelescopeParams.GRAVITY_COMP_POWER;

        if (elbow != null)
        {
            // There is an elbow, compensation should take into account of elbow angle.
            // This is not a correct equation, it needs to be updated to reflect the physical mechanism.
            gravityCompPower *= Math.cos(Math.toRadians(elbow.getPosition()));
        }

        return gravityCompPower;
    }   //getTelescopeGravityComp

    /**
     * This method calculates the power required to make the elbow gravity neutral.
     *
     * @param motor specifies the motor for determining its gravity comp power.
     * @param currPower specifies the current applied PID power (not used).
     * @return calculated compensation power.
     */
    private double getElbowGravityComp(TrcMotor motor, double currPower)
    {
        double gravityCompPower = tuneElbowGravityCompPower != null?
            tuneElbowGravityCompPower: ElbowParams.GRAVITY_COMP_POWER;
        // This is not a correct equation, it needs to be updated to reflect the physical mechanism.
        gravityCompPower *= telescope.getPosition()*Math.cos(Math.toRadians(elbow.getPosition()));

        return gravityCompPower;
    }   //getElbowGravityComp

    /**
     * This method sets the telescope arm position including extending the telescope and tilting the arm.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param telescopePos specifies the telescope extend position, can be null if not changing telescope position.
     * @param elbowPos specifies the elbow angle, can be null if not changing elbow position.
     * @param completionEvent specifies an event to signal when completion, can be null if not provided.
     * @param timeout specifies timeout in seconds, can be zero if no timeout.
     */
    public void setPosition(
        String owner, Double telescopePos, Double elbowPos, TrcEvent completionEvent, double timeout)
    {
        TrcEvent telescopeEvent = null;
        TrcEvent elbowEvent = null;
        TrcEvent timeoutEvent = null;

        if (telescopePos != null)
        {
            telescopeEvent = new TrcEvent(SUBSYSTEM_NAME + ".telescopeEvent");
            telescope.setPosition(owner, 0.0, telescopePos, true, TelescopeParams.POWER_LIMIT, telescopeEvent, 0.0);
        }

        if (elbow != null && elbowPos != null)
        {
            elbowEvent = new TrcEvent(SUBSYSTEM_NAME + ".elbowEvent");
            elbow.setPosition(owner, 0.0, elbowPos, true, ElbowParams.POWER_LIMIT, elbowEvent, 0.0);
        }

        if (timeout > 0.0 && (telescopeEvent != null || elbowEvent != null))
        {
            timeoutEvent = new TrcEvent(SUBSYSTEM_NAME + ".timeoutEvent");
            timeoutEvent.setCallback(this::setPositionTimeout, null);
            timer.set(timeout, timeoutEvent);
        }

        if (completionEvent != null && (telescopeEvent != null || elbowEvent != null))
        {
            completionEvent.signalOnEvents(true, false, telescopeEvent, elbowEvent);
        }
    }   //setPosition

    /**
     * This method is called when the setPosition operation is timed out.
     *
     * @param context not used.
     * @param canceled not used.
     */
    private void setPositionTimeout(Object context, boolean canceled)
    {
        // By canceling both the telescope and the elbow, it will cause telescopeEvent and elbowEvent to be canceled
        // as well. That in turn will cause completionEvent to be canceled.
        telescope.cancel();
        if (elbow != null)
        {
            elbow.cancel();
        }
    }   //setPositionTimeout

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
        if (elbow != null)
        {
            elbow.cancel();
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
        TrcEvent elbowEvent = null;

        telescope.zeroCalibrate(
            owner, TelescopeParams.ZERO_CAL_POWER, telescopeEvent, TelescopeParams.ZERO_CAL_TIMEOUT);

        if (elbow != null)
        {
            elbowEvent = new TrcEvent(SUBSYSTEM_NAME + ".elbowEvent");
            elbow.zeroCalibrate(owner, ElbowParams.ZERO_CAL_POWER, elbowEvent, ElbowParams.ZERO_CAL_TIMEOUT);
        }

        if (completionEvent != null)
        {
            completionEvent.signalOnEvents(true, false, telescopeEvent, elbowEvent);
        }
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        telescope.setPosition(
            TelescopeParams.TURTLE_DELAY, TelescopeParams.TURTLE_POS, true, TelescopeParams.POWER_LIMIT);

        if (elbow != null)
        {
            elbow.setPosition(
                ElbowParams.TURTLE_DELAY, ElbowParams.TURTLE_POS, true, ElbowParams.POWER_LIMIT);
        }
    }   //resetState

    private static final String DBKEY_TELESCOPE_PWR_INFO    = SUBSYSTEM_NAME + "/TelescopePwrInfo";     //String
    private static final String DBKEY_TELESCOPE_POS_INFO    = SUBSYSTEM_NAME + "/TelescopePosInfo";     //String
    private static final String DBKEY_TELESCOPE_LOWER_LIMIT = SUBSYSTEM_NAME + "/TelescopeLowerLimit";  //Boolean
    private static final String DBKEY_TELESCOPE_UPPER_LIMIT = SUBSYSTEM_NAME + "/TelescopeUpperLimit";  //Boolean
    private static final String DBKEY_ELBOW_PWR_INFO        = SUBSYSTEM_NAME + "/ElbowPwrInfo";         //String
    private static final String DBKEY_ELBOW_POS_INFO        = SUBSYSTEM_NAME + "/ElbowPosInfo";         //String
    private static final String DBKEY_ELBOW_LOWER_LIMIT     = SUBSYSTEM_NAME + "/ElbowLowerLimit";      //Boolean
    private static final String DBKEY_ELBOW_UPPER_LIMIT     = SUBSYSTEM_NAME + "/ElbowUpperLimit";      //Boolean

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
        dashboard.refreshKey(DBKEY_ELBOW_PWR_INFO, "");
        dashboard.refreshKey(DBKEY_ELBOW_POS_INFO, "");
        dashboard.refreshKey(DBKEY_ELBOW_LOWER_LIMIT, false);
        dashboard.refreshKey(DBKEY_ELBOW_UPPER_LIMIT, false);
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

            if (TelescopeParams.HAS_LOWER_LIMIT_SW)
            {
                dashboard.putBoolean(DBKEY_TELESCOPE_LOWER_LIMIT, telescope.isLowerLimitSwitchActive());
            }

            if (TelescopeParams.HAS_UPPER_LIMIT_SW)
            {
                dashboard.putBoolean(DBKEY_TELESCOPE_UPPER_LIMIT, telescope.isUpperLimitSwitchActive());
            }

            if (elbow != null)
            {
                dashboard.putString(DBKEY_ELBOW_PWR_INFO, elbow.getPower() + "/" + elbow.getCurrent());
                dashboard.putString(DBKEY_ELBOW_POS_INFO, elbow.getPosition() + "/" + elbow.getPidTarget());

                if (ElbowParams.HAS_LOWER_LIMIT_SW)
                {
                    dashboard.putBoolean(DBKEY_ELBOW_LOWER_LIMIT, elbow.isLowerLimitSwitchActive());
                }

                if (ElbowParams.HAS_UPPER_LIMIT_SW)
                {
                    dashboard.putBoolean(DBKEY_ELBOW_UPPER_LIMIT, elbow.isUpperLimitSwitchActive());
                }
            }
        }
        // The following entries need to be updated at fast rate for plotting graphs.
        if (tuneSubsystemName != null)
        {
            if (tuneSubsystemName.equalsIgnoreCase(TelescopeParams.MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, telescope.getPosition());
            }
            else if (elbow != null && tuneSubsystemName.equalsIgnoreCase(ElbowParams.MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_INPUT, elbow.getPosition());
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
        if (subsystemName.equalsIgnoreCase(TelescopeParams.MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(TelescopeParams.posPidCoeffs)
                    .setPidControlParams(TelescopeParams.POS_PID_TOLERANCE, TelescopeParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, TelescopeParams.MIN_POS);
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, TelescopeParams.GRAVITY_COMP_POWER);
        }
        else if (elbow != null && subsystemName.equalsIgnoreCase(ElbowParams.MOTOR_NAME))
        {
            FrcTest.testChoices.setSubsystemPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(ElbowParams.posPidCoeffs)
                    .setPidControlParams(ElbowParams.POS_PID_TOLERANCE, ElbowParams.USE_SOFTWARE_PID));
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, ElbowParams.MIN_POS);
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, ElbowParams.GRAVITY_COMP_POWER);
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
        if (subsystemName.equalsIgnoreCase(TelescopeParams.MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, TelescopeParams.MIN_POS);
            tuneTelescopeGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, TelescopeParams.GRAVITY_COMP_POWER);
            telescope.setPositionPidParameters(pidParams, null);
            telescope.setPosition(target);
            gravityPower = tuneTelescopeGravityCompPower;
            tuneSubsystemName = subsystemName;
        }
        else if (elbow != null && subsystemName.equalsIgnoreCase(ElbowParams.MOTOR_NAME))
        {
            target = dashboard.getNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, ElbowParams.MIN_POS);
            tuneElbowGravityCompPower = dashboard.getNumber(
                FrcTest.DBKEY_SUBSYSTEM_GRAVITY_POWER, ElbowParams.GRAVITY_COMP_POWER);
            elbow.setPositionPidParameters(pidParams, null);
            elbow.setPosition(target);
            gravityPower = tuneElbowGravityCompPower;
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

        if (subsystemName.equalsIgnoreCase(TelescopeParams.MOTOR_NAME))
        {
            target = telescope.presetPositionUp(null, null);
        }
        else if (elbow != null && subsystemName.equalsIgnoreCase(ElbowParams.MOTOR_NAME))
        {
            target = elbow.presetPositionUp(null, null);
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

        if (subsystemName.equalsIgnoreCase(TelescopeParams.MOTOR_NAME))
        {
            target = telescope.presetPositionDown(null, null);
        }
        else if (elbow != null && subsystemName.equalsIgnoreCase(ElbowParams.MOTOR_NAME))
        {
            target = elbow.presetPositionDown(null, null);
        }

        if (target != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET, target);
            telescope.tracer.traceInfo(instanceName, "Tune %s Down: target=%.3f", subsystemName, target);
        }
    }   //setNextTuneTargetDown

}   //class TelescopeArm
