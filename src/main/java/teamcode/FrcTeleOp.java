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

package teamcode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcXboxController;
import teamcode.subsystems.DriveBase;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.dataprocessor.TrcWarpSpace;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.drivebase.TrcSwerveDrive;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcTeleOp.class.getSimpleName();
    protected static final boolean traceButtonEvents = true;

    private static final String DBKEY_PREFIX            = "TeleOp/";
    private static final String DBKEY_DRIVE_MODE        = DBKEY_PREFIX + "DriveMode";           //Choices
    private static final String DBKEY_DRIVE_ORIENTATION = DBKEY_PREFIX + "DriveOrientation";    //Choices
    private static final String DBKEY_DRIVE_NORMAL_SCALE= DBKEY_PREFIX + "DriveNormalScale";    //Number
    private static final String DBKEY_DRIVE_SLOW_SCALE  = DBKEY_PREFIX + "DriveSlowScale";      //Number
    private static final String DBKEY_TURN_NORMAL_SCALE = DBKEY_PREFIX + "TurnNormalScale";     //Number
    private static final String DBKEY_TURN_SLOW_SCALE   = DBKEY_PREFIX + "TurnSlowScale";       //Number
    private static final String DBKEY_USE_RUMBLE        = DBKEY_PREFIX + "UseRumble";           //Boolean

    public static final double DEF_DRIVE_NORMAL_SCALE = 1.0;
    public static final double DEF_DRIVE_SLOW_SCALE = 0.2;
    public static final double DEF_TURN_NORMAL_SCALE = 0.75;
    public static final double DEF_TURN_SLOW_SCALE = 0.2;
    //
    // Global objects.
    //
    protected final FrcDashboard dashboard;
    protected final Robot robot;
    private final FrcChoiceMenu<DriveMode> driveModeMenu;
    private final FrcChoiceMenu<DriveOrientation> driveOrientationMenu;

    private double drivePowerScale;
    private double turnPowerScale;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    protected boolean controlsEnabled = false;
    // Locked heading
    private final TrcPidController turnPidCtrl;
    private Double lockedHeading;
    private boolean rumbling = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.dashboard = FrcDashboard.getInstance();
        this.robot = robot;

        driveModeMenu = new FrcChoiceMenu<>(DBKEY_DRIVE_MODE);
        driveModeMenu.addChoice(DriveMode.Tank.name(), DriveMode.Tank);
        driveModeMenu.addChoice(DriveMode.Holonomic.name(), DriveMode.Holonomic);
        driveModeMenu.addChoice(DriveMode.Arcade.name(), DriveMode.Arcade, true, true);

        driveOrientationMenu = new FrcChoiceMenu<>(DBKEY_DRIVE_ORIENTATION);
        driveOrientationMenu.addChoice(DriveOrientation.Inverted.name(), DriveOrientation.Inverted);
        driveOrientationMenu.addChoice(DriveOrientation.Robot.name(), DriveOrientation.Robot);
        driveOrientationMenu.addChoice(DriveOrientation.Field.name(), DriveOrientation.Field, true, true);

        dashboard.refreshKey(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
        dashboard.refreshKey(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
        drivePowerScale = dashboard.getNumber(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        turnPowerScale = dashboard.getNumber(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_USE_RUMBLE, RobotParams.Preferences.useRumble);

        turnPidCtrl = robot.robotBase != null && robot.robotBase.purePursuitDrive != null?
            robot.robotBase.purePursuitDrive.getTurnPidCtrl(): null;
        lockedHeading = null;
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling gamepads.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotBase != null)
        {
            // Set robot to FIELD by default but don't change the heading.
            robot.robotDriveBase.setDriveOrientation(driveOrientationMenu.getCurrentChoiceObject(), false);
        }
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling gamepads.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase subsystem.
                //
                if (robot.robotBase != null)
                {
                    if (robot.driverController != null)
                    {
                        double[] inputs = robot.driverController.getDriveInputs(
                            driveModeMenu.getCurrentChoiceObject(), true, drivePowerScale, turnPowerScale,
                            lockedHeading != null);
                        // inputs have changed or rotating to lockedHeading.
                        if (inputs != null)
                        {
                            if (turnPidCtrl != null && lockedHeading != null)
                            {
                                if (inputs[2] == 0.0)
                                {
                                    // No turning movement from joystick, use PID to turn to lockedHeading.
                                    double currHeading = robot.robotBase.driveBase.getHeading();
                                    double targetHeading = TrcWarpSpace.getOptimizedTarget(
                                        lockedHeading, currHeading, 360.0);

                                    if (Math.abs(targetHeading - currHeading) >
                                        robot.robotInfo.baseParams.turnPidTolerance)
                                    {
                                        inputs[2] = TrcUtil.clipRange(
                                            turnPidCtrl.calculate(currHeading, lockedHeading),
                                            robot.robotInfo.baseParams.turnPowerLimit);
                                        robot.globalTracer.traceDebug(
                                            moduleName,
                                            "currHeading=%f, lockedHeading=%f, targetHeading=%f, turnPower=%f",
                                            currHeading, lockedHeading, targetHeading, inputs[2]);
                                    }
                                    else
                                    {
                                        // lockedHeading target reached, cancel.
                                        lockedHeading = null;
                                    }
                                }
                                else
                                {
                                    // Driver is rotating the robot, cancel lockedHeading.
                                    lockedHeading = null;
                                }
                            }

                            robot.robotDriveBase.subsystemControl(driverAltFunc, inputs);
                        }


                        if (robot.dashboard.getBoolean(DBKEY_USE_RUMBLE, RobotParams.Preferences.useRumble))
                        {
                            if (!rumbling &&
                                elapsedTime > RobotParams.Game.TELEOP_PERIOD - RobotParams.Game.ENDGAME_THRESHOLD)
                            {
                                robot.driverController.setRumble(RumbleType.kBothRumble, 1.0, 0.5);
                                rumbling = true;
                            }
                        }
                    }
                }
                //
                // Other subsystems.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    // Analog control of subsystems.
                }
            }
        }
    }   //periodic

    /**
     * This method enables/disables gamepad controls.
     *
     * @param enabled specifies true to enable gamepad controls, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (robot.driverController != null)
        {
            robot.driverController.setButtonEventHandler(enabled? this::driverButtonEvent: null);
        }

        if (robot.operatorController != null)
        {
            robot.operatorController.setButtonEventHandler(enabled? this::operatorButtonEvent: null);
        }
    }   //setControlsEnabled

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Driver: " + button + "=" + (pressed? "pressed": "released"));
        switch (button)
        {
            case A:
                break;

            case B:
                if (robot.robotDriveBase != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            robot.robotDriveBase.subsystemAction(DriveBase.Action.ToggleGyroAssist, null);
                        }
                        else
                        {
                            robot.robotDriveBase.subsystemAction(DriveBase.Action.ToggleDriveMode, null);
                        }
                    }
                }
                break;

            case X:
                // Turtle mode (alt-func: X-Mode).
                if (pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotBase != null)
                        {
                            ((TrcSwerveDrive) (robot.robotBase.driveBase)).setXMode(null);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> X Mode");
                        }
                    }
                    else
                    {
                        robot.turtle();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Turtle Mode.");
                    }
                }
                break;

            case Y:
                break;

            case LeftBumper:
                driverAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + driverAltFunc);
                break;

            case RightBumper:
                setDriveSpeedMode(pressed, driverAltFunc);
                break;

            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All");
                    if (!driverAltFunc)
                    {
                        robot.zeroCalibrate(null, null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Zero Calibrate");
                    }
                }
                break;

            case Start:
                if (pressed)
                {
                    robot.setRelocalizationMode(
                        driverAltFunc? Robot.RelocalizationMode.Continuous: Robot.RelocalizationMode.OneShot);
                }
                break;

            default:
                break;
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Operator: " + button + "=" + (pressed? "pressed": "released"));
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + operatorAltFunc);
                break;

            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.zeroCalibrate(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All and Zero Calibrate");
                }
                break;

            case Start:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All");
                }
                break;

            default:
                break;
        }
    }   //operatorButtonEvent

    /**
     * This method is called to set drive speed modes.
     *
     * @param pressed specifies true if the button is pressed, false if released.
     * @param altFunc specifies true if AltFunc is pressed, false otherwise.
     */
    private void setDriveSpeedMode(boolean pressed, boolean altFunc)
    {
        if (!altFunc)
        {
            // Press and hold for slow drive.
            if (pressed)
            {
                drivePowerScale = robot.dashboard.getNumber(DBKEY_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
                turnPowerScale = robot.dashboard.getNumber(DBKEY_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower slow.");
            }
            else
            {
                drivePowerScale = robot.dashboard.getNumber(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
                turnPowerScale = robot.dashboard.getNumber(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
                robot.globalTracer.traceInfo(moduleName, ">>>>> DrivePower normal.");
            }
        }
    }   //setDriveSpeedMode

}   //class FrcTeleOp
