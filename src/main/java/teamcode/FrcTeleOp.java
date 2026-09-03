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
import teamcode.subsystems.CrServoArm;
import teamcode.subsystems.Elevator;
import teamcode.subsystems.Intake;
import teamcode.subsystems.MotorArm;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.TelescopeArm;
import teamcode.subsystems.Turret;
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
    private static final String DBKEY_SHOW_DRIVE_POWER  = DBKEY_PREFIX + "ShowDrivePower";      //Boolean
    private static final String DBKEY_DRIVE_PWR_INFO    = DBKEY_PREFIX + "DrivePwrInfo";        //String
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

    private double driveSpeedScale;
    private double turnSpeedScale;
    private boolean controlsEnabled = false;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    // Locked heading
    private final TrcPidController turnPidCtrl;
    private Double lockedHeading;
    private boolean rumbling = false;

    private double prevMotorArmPower = 0.0;
    private double prevServoArmPower = 0.0;
    private double prevTelescopePower = 0.0;
    private double prevTelescopeElbowPower = 0.0;
    private double prevElevatorPower = 0.0;
    private double prevTurretPower = 0.0;
    private double prevDiffyWristTiltPower = 0.0;
    private double prevDiffyWristRotatePower = 0.0;
    private double prevServoWristPower = 0.0;
    private double prevServoExtenderPower = 0.0;
    private double prevLatchPower = 0.0;
    private boolean extenderExtended = false;

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
        driveSpeedScale = dashboard.getNumber(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        turnSpeedScale = dashboard.getNumber(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
        dashboard.refreshKey(DBKEY_SHOW_DRIVE_POWER, false);
        dashboard.refreshKey(DBKEY_DRIVE_PWR_INFO, "");
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
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotBase != null)
        {
            // Set robot to FIELD by default but don't change the heading.
            robot.setDriveOrientation(driveOrientationMenu.getCurrentChoiceObject(), false);
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
        // Disabling joysticks.
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
                        boolean showDriveBaseStatus = robot.dashboard.getBoolean(DBKEY_SHOW_DRIVE_POWER, false);
                        double[] driveInputs = robot.driverController.getDriveInputs(
                            driveModeMenu.getCurrentChoiceObject(), true, driveSpeedScale, turnSpeedScale,
                            lockedHeading != null);
                        // driveInputs have changed or rotating to lockedHeading.
                        if (driveInputs != null)
                        {
                            double turnPower = driveInputs[2];

                            if (turnPidCtrl != null && lockedHeading != null)
                            {
                                if (turnPower == 0.0)
                                {
                                    double currHeading = robot.robotBase.driveBase.getHeading();
                                    double targetHeading = TrcWarpSpace.getOptimizedTarget(lockedHeading, currHeading, 360.0);

                                    if (Math.abs(targetHeading - currHeading) >
                                        robot.robotInfo.baseParams.turnPidTolerance)
                                    {
                                        turnPower = TrcUtil.clipRange(
                                            turnPidCtrl.calculate(currHeading, lockedHeading),
                                            robot.robotInfo.baseParams.turnPowerLimit);
                                        robot.globalTracer.traceDebug(
                                            moduleName,
                                            "currHeading=%f, lockedHeading=%f, targetHeading=%f, turnPower=%f",
                                            currHeading, lockedHeading, targetHeading, turnPower);
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

                            if (robot.robotBase.driveBase.supportsHolonomicDrive())
                            {
                                Double gyroAngle = robot.robotBase.driveBase.getDriveGyroAngle();

                                robot.robotBase.driveBase.holonomicDrive(
                                    null, driveInputs[0], driveInputs[1], turnPower, gyroAngle);
                                if (showDriveBaseStatus)
                                {
                                    robot.dashboard.putString(
                                        DBKEY_DRIVE_PWR_INFO,
                                        String.format(
                                            "Holonomic: x=%.2f, y=%.2f, rot=%.2f, gyroAngle=%.2f",
                                            driveInputs[0], driveInputs[1], turnPower, gyroAngle));
                                }
                            }
                            else
                            {
                                robot.robotBase.driveBase.arcadeDrive(driveInputs[1], turnPower);
                                if (showDriveBaseStatus)
                                {
                                    robot.dashboard.putString(
                                        DBKEY_DRIVE_PWR_INFO,
                                        String.format(
                                            "Arcade: x=%.2f, y=%.2f, rot=%.2f",
                                            driveInputs[0], driveInputs[1], turnPower));
                                }
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
                    // Note that this sample code assumes only one subsystem is enabled at a time for demo purpose.
                    // Therefore, the same control may be assigned to multiple subsystems.
                    if (robot.motorArm != null)
                    {
                        double armPower = robot.driverController.getLeftStickY(true);
                        if (armPower != prevMotorArmPower)
                        {
                            if (driverAltFunc)
                            {
                                // Manual override.
                                robot.motorArm.setPower(armPower);
                            }
                            else
                            {
                                robot.motorArm.setPidPower(
                                    armPower, MotorArm.Params.POWER_LIMIT, MotorArm.Params.MIN_POS,
                                    MotorArm.Params.MAX_POS, true);
                            }
                            prevMotorArmPower = armPower;
                        }
                    }
                    else if (robot.crServoArm != null)
                    {
                        double armPower = robot.driverController.getLeftStickY(true);
                        if (armPower != prevServoArmPower)
                        {
                            if (driverAltFunc)
                            {
                                // Manual override.
                                robot.crServoArm.setPower(armPower);
                            }
                            else
                            {
                                robot.crServoArm.setPidPower(
                                    armPower, CrServoArm.Params.POWER_LIMIT, CrServoArm.Params.MIN_POS,
                                    CrServoArm.Params.MAX_POS, true);
                            }
                            prevServoArmPower = armPower;
                        }
                    }
                    else if (robot.telescopeArm != null)
                    {
                        double telescopePower = robot.driverController.getLeftStickY(true);
                        if (telescopePower != prevTelescopePower)
                        {
                            if (driverAltFunc)
                            {
                                // Manual override.
                                robot.telescopeArm.telescope.setPower(telescopePower);
                            }
                            else
                            {
                                robot.telescopeArm.telescope.setPidPower(
                                    telescopePower, TelescopeArm.TelescopeParams.POWER_LIMIT,
                                    TelescopeArm.TelescopeParams.MIN_POS, TelescopeArm.TelescopeParams.MAX_POS,
                                    true);
                            }
                            prevTelescopePower = telescopePower;
                        }

                        if (robot.telescopeArm.elbow != null)
                        {
                            double elbowPower = robot.driverController.getRightStickY(true);
                            if (elbowPower != prevTelescopeElbowPower)
                            {
                                if (driverAltFunc)
                                {
                                    // Manual override.
                                    robot.telescopeArm.elbow.setPower(elbowPower);
                                }
                                else
                                {
                                    robot.telescopeArm.elbow.setPidPower(
                                        elbowPower, TelescopeArm.ElbowParams.POWER_LIMIT,
                                        TelescopeArm.ElbowParams.MIN_POS, TelescopeArm.ElbowParams.MAX_POS,
                                        true);
                                }
                                prevTelescopeElbowPower = elbowPower;
                            }
                        }
                    }
                    else if (robot.elevator != null)
                    {
                        double elevatorPower = robot.driverController.getLeftStickY(true);
                        if (elevatorPower != prevElevatorPower)
                        {
                            if (driverAltFunc)
                            {
                                // Manual override.
                                robot.elevator.setPower(elevatorPower);
                            }
                            else
                            {
                                robot.elevator.setPidPower(
                                    elevatorPower, Elevator.Params.POWER_LIMIT, Elevator.Params.MIN_POS,
                                    Elevator.Params.MAX_POS, true);
                            }
                            prevElevatorPower = elevatorPower;
                        }
                    }
                    else if (robot.turret != null)
                    {
                        double turretPower = robot.driverController.getLeftStickY(true);
                        if (turretPower != prevTurretPower)
                        {
                            if (driverAltFunc)
                            {
                                // Manual override.
                                robot.turret.setPower(turretPower);
                            }
                            else
                            {
                                robot.turret.setPidPower(
                                    turretPower, Turret.Params.POWER_LIMIT, Turret.Params.MIN_POS,
                                    Turret.Params.MAX_POS, true);
                            }
                            prevTurretPower = turretPower;
                        }
                    }
                    else if (robot.diffyWrist != null)
                    {
                        double rotatePower = robot.driverController.getLeftStickX(true);
                        double tiltPower = robot.driverController.getLeftStickY(true);
                        if (rotatePower != prevDiffyWristRotatePower || tiltPower != prevDiffyWristTiltPower)
                        {
                            robot.diffyWrist.wrist.setPower(tiltPower, rotatePower);
                            prevDiffyWristRotatePower = rotatePower;
                            prevDiffyWristTiltPower = tiltPower;
                        }
                    }
                    else if (robot.servoWrist != null)
                    {
                        double wristPower = robot.driverController.getLeftStickY(true);
                        if (wristPower != prevServoWristPower)
                        {
                            robot.servoWrist.setPower(wristPower);
                            prevServoWristPower = wristPower;
                        }
                    }
                    else if (robot.servoExtender != null)
                    {
                        double extenderPower = robot.driverController.getLeftStickY(true);
                        if (extenderPower != prevServoExtenderPower)
                        {
                            robot.servoExtender.setPower(extenderPower);
                            prevServoExtenderPower = extenderPower;
                        }
                    }
                    else if (robot.latch != null)
                    {
                        double latchPower = robot.driverController.getLeftStickY(true);
                        if (latchPower != prevLatchPower)
                        {
                            robot.latch.setPower(latchPower);
                            prevLatchPower = latchPower;
                        }
                    }
                }

                if (robot.dashboard.getBoolean(DBKEY_USE_RUMBLE, RobotParams.Preferences.useRumble))
                {
                    if (!rumbling && elapsedTime > RobotParams.Game.TELEOP_PERIOD - RobotParams.Game.ENDGAME_THRESHOLD)
                    {
                        robot.driverController.setRumble(RumbleType.kBothRumble, 1.0, 0.5);
                        rumbling = true;
                    }
                }
            }
        }
    }   //periodic

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (robot.driverController != null)
        {
            robot.driverController.setButtonEventHandler(enabled? this::driverControllerButtonEvent: null);
        }

        if (robot.operatorController != null)
        {
            robot.operatorController.setButtonEventHandler(enabled? this::operatorControllerButtonEvent: null);
        }
    }   //setControlsEnabled

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                if (robot.shooter != null)
                {
                    if (pressed)
                    {
                        if (robot.autoShootTask != null)
                        {
                            // Auto Shoot Task is enabled, auto shoot at any AprilTag detected.
                            if (robot.autoShootTask.isActive())
                            {
                                robot.autoShootTask.cancel();
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Auto Shoot");
                            }
                            else
                            {
                                robot.autoShootTask.autoShoot(moduleName, null, !driverAltFunc, null);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Shoot");
                            }
                        }
                        else
                        {
                            // Auto Shoot Task is disabled, shoot manually.
                            if (robot.shooter.isActive())
                            {
                                robot.shooter.cancel(moduleName);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Manual Shoot");
                            }
                            else
                            {
                                robot.shooter.aimShooter(
                                    moduleName, robot.shooterSubsystem.shooter1Velocity.getValue(), 0.0,
                                    null, null, null, 0.0, robot.shooterSubsystem::shoot, null,
                                    Shooter.ShooterMotorParams.OFF_DELAY);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Manual Shoot");
                            }
                        }
                    }
                }
                else if (robot.intake != null)
                {
                    if (pressed)
                    {
                        if (robot.autoPickupTask != null)
                        {
                            if (robot.autoPickupTask.isActive())
                            {
                                robot.autoPickupTask.cancel();
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Auto Pickup");
                            }
                            else
                            {
                                robot.autoPickupTask.autoPickup(
                                    moduleName, null, FrcAuto.autoChoices.alliance, !driverAltFunc);
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Auto Pickup (useVision=" + !driverAltFunc + ")");
                            }
                        }
                        else
                        {
                            if (driverAltFunc)
                            {
                                if (robot.intake.getPower() == 0.0)
                                {
                                    robot.intake.setPower(Intake.Params.INTAKE_POWER);
                                    robot.globalTracer.traceInfo(moduleName, ">>>>> Manual Intake");
                                }
                                else
                                {
                                    robot.intake.cancel();
                                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Manual Intake");
                                }
                            }
                            else
                            {
                                if (robot.intake.isAutoActive())
                                {
                                    robot.intake.cancel();
                                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Sensor Intake");
                                }
                                else
                                {
                                    robot.intake.autoIntake(moduleName);
                                    robot.globalTracer.traceInfo(moduleName, ">>>>> Sensor Intake");
                                }
                            }
                        }
                    }
                }
                else if (robot.servoExtender != null)
                {
                    if (pressed)
                    {
                        extenderExtended = !extenderExtended;
                        if (extenderExtended)
                        {
                            robot.servoExtenderSubsystem.extend();
                        }
                        else
                        {
                            robot.servoExtenderSubsystem.retract();
                        }
                    }
                }
                else if (robot.servoClaw != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            if (robot.servoClaw.isClosed())
                            {
                                robot.servoClaw.open();
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Opening claws");
                            }
                            else
                            {
                                robot.servoClaw.close();
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Closing claws");
                            }
                        }
                        else
                        {
                            if (robot.servoClaw.isAutoActive() || robot.servoClaw.hasObject())
                            {
                                robot.servoClaw.cancel();
                                robot.servoClaw.open();
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Canceling AutoGrab.");
                            }
                            else
                            {
                                robot.servoClaw.autoGrab(null, 0.0, null, 0.0);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Enabling AutoGrab.");
                            }
                        }
                    }
                }
                break;

            case B:
                // Toggle between field or robot oriented driving.
                if (robot.robotBase != null && pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotBase.driveBase.getDriveOrientation() != DriveOrientation.Field)
                        {
                            robot.setDriveOrientation(DriveOrientation.Field, true);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Field");
                        }
                        else
                        {
                            robot.setDriveOrientation(DriveOrientation.Robot, false);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Robot");
                        }
                    }
                    else
                    {
                        robot.robotBase.driveBase.resetFieldForwardHeading();
                        robot.globalTracer.traceInfo(
                            moduleName,
                            ">>>>> Reset field forward heading (heading=" + robot.robotBase.driveBase.getHeading() +
                            ")");
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
                if (pressed)
                {
                    driveSpeedScale = robot.dashboard.getNumber(
                        DBKEY_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(
                        DBKEY_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Slow Drive");
                }
                else
                {
                    driveSpeedScale = robot.dashboard.getNumber(
                        DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(
                        DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Normal Drive");
                }
                break;

            case DpadUp:
                if (robot.motorArm != null)
                {
                    if (pressed)
                    {
                        robot.motorArm.presetPositionUp(null, MotorArm.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> MotorArm position up");
                    }
                }
                else if (robot.crServoArm != null)
                {
                    if (pressed)
                    {
                        robot.crServoArm.presetPositionUp(null, CrServoArm.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> CrServoArm position up");
                    }
                }
                else if (robot.telescopeArm != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            if (robot.telescopeArm.elbow != null)
                            {
                                robot.telescopeArm.elbow.presetPositionUp(
                                    null, TelescopeArm.ElbowParams.POWER_LIMIT);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Telescope elbow position up");
                            }
                        }
                        else
                        {
                            robot.telescopeArm.telescope.presetPositionUp(
                                null, TelescopeArm.TelescopeParams.POWER_LIMIT);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Telescope position up");
                        }
                    }
                }
                else if (robot.elevator != null)
                {
                    if (pressed)
                    {
                        robot.elevator.presetPositionUp(null, Elevator.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Elevator position up");
                    }
                }
                else if (robot.turret != null)
                {
                    if (pressed)
                    {
                        robot.turret.presetPositionUp(null, Turret.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Turret position up");
                    }
                }
                else if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.tiltPresetPositionUp(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> DiffyWristTilt position up");
                    }
                }
                else if (robot.servoWrist != null)
                {
                    if (pressed)
                    {
                        robot.servoWrist.presetPositionUp(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> ServoWrist position up");
                    }
                }
                else if (robot.servoExtender != null)
                {
                    if (pressed)
                    {
                        robot.servoExtender.presetPositionUp(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> ServoExtender position up");
                    }
                }
                else if (robot.latch != null)
                {
                    if (pressed)
                    {
                        robot.latch.presetPositionUp(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Latch position up");
                    }
                }
                else if (robot.shooter != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.shooter1Velocity.upValue();
                        robot.dashboard.putNumber(
                            FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET,
                            robot.shooterSubsystem.shooter1Velocity.getValue());
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Shooter velocity up");
                    }
                }
                break;

            case DpadDown:
                if (robot.motorArm != null)
                {
                    if (pressed)
                    {
                        robot.motorArm.presetPositionDown(null, MotorArm.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> MotorArm position down");
                    }
                }
                else if (robot.crServoArm != null)
                {
                    if (pressed)
                    {
                        robot.crServoArm.presetPositionDown(null, CrServoArm.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> CrServoArm position down");
                    }
                }
                else if (robot.telescopeArm != null)
                {
                    if (pressed)
                    {
                        if (driverAltFunc)
                        {
                            if (robot.telescopeArm.elbow != null)
                            {
                                robot.telescopeArm.elbow.presetPositionDown(
                                    null, TelescopeArm.ElbowParams.POWER_LIMIT);
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Telescope elbow position down");
                            }
                        }
                        else
                        {
                            robot.telescopeArm.telescope.presetPositionDown(
                                null, TelescopeArm.TelescopeParams.POWER_LIMIT);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Telescope position down");
                        }
                    }
                }
                else if (robot.elevator != null)
                {
                    if (pressed)
                    {
                        robot.elevator.presetPositionDown(null, Elevator.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Elevator position down");
                    }
                }
                else if (robot.turret != null)
                {
                    if (pressed)
                    {
                        robot.turret.presetPositionDown(null, Turret.Params.POWER_LIMIT);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Turret position down");
                    }
                }
                else if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.tiltPresetPositionDown(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> DiffyWristTilt position down");
                    }
                }
                else if (robot.servoWrist != null)
                {
                    if (pressed)
                    {
                        robot.servoWrist.presetPositionDown(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> ServoWrist position down");
                    }
                }
                else if (robot.servoExtender != null)
                {
                    if (pressed)
                    {
                        robot.servoExtender.presetPositionDown(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> ServoExtender position down");
                    }
                }
                else if (robot.latch != null)
                {
                    if (pressed)
                    {
                        robot.latch.presetPositionDown(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Latch position down");
                    }
                }
                else if (robot.shooter != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.shooter1Velocity.downValue();
                        robot.dashboard.putNumber(
                            FrcTest.DBKEY_SUBSYSTEM_TUNE_TARGET,
                            robot.shooterSubsystem.shooter1Velocity.getValue());
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Shooter velocity down");
                    }
                }
                break;

            case DpadLeft:
                if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.rotatePresetPositionDown(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> DiffyWristRotate position down");
                    }
                }
                else if (robot.shooter != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.shooter1Velocity.downIncrement();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Shooter velocity increment down");
                    }
                }
                break;

            case DpadRight:
                if (robot.diffyWrist != null)
                {
                    if (pressed)
                    {
                        robot.diffyWrist.rotatePresetPositionUp(null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> DiffyWristRotate position up");
                    }
                }
                else if (robot.shooter != null)
                {
                    if (pressed)
                    {
                        robot.shooterSubsystem.shooter1Velocity.upIncrement();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Shooter velocity increment up");
                    }
                }
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
                break;

            default:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "OperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

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
    }   //operatorControllerButtonEvent

}   //class FrcTeleOp
