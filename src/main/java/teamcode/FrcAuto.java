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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcUserChoices;
import teamcode.autocommands.CmdAuto;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in Autonomous Mode.
 */
public class FrcAuto implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcAuto.class.getSimpleName();

    private static final String DBKEY_PREFIX            = "Auto/";
    private static final String DBKEY_ALLIANCE          = DBKEY_PREFIX + "Alliance";        //Choices
    private static final String DBKEY_START_POS         = DBKEY_PREFIX + "StartPos";        //Choices
    private static final String DBKEY_STRATEGY          = DBKEY_PREFIX + "Strategy";        //Choices
    private static final String DBKEY_START_DELAY       = DBKEY_PREFIX + "StartDelay";      //Number
    private static final String DBKEY_X_DRIVE_TARGET    = DBKEY_PREFIX + "XDriveTarget";    //Number
    private static final String DBKEY_Y_DRIVE_TARGET    = DBKEY_PREFIX + "YDriveTarget";    //Number
    private static final String DBKEY_TURN_TARGET       = DBKEY_PREFIX + "TurnTarget";      //Number
    private static final String DBKEY_DRIVE_POWER       = DBKEY_PREFIX + "DrivePower";      //Number
    private static final String DBKEY_DRIVE_TIME        = DBKEY_PREFIX + "DriveTime";       //Number
    private static final String DBKEY_PATHFILE          = DBKEY_PREFIX + "PathFile";        //String
    // Game specific options

    public static final String DBKEY_CHOICES_REFRESH    = DBKEY_PREFIX + "ChoicesRefresh";  //Boolean
    //
    // Auto choices enums.
    //
    public enum AutoStartPos
    {
        StartPos1(0),
        StartPos2(1),
        StartPos3(2);
        // The value can be used as index into arrays if necessary.
        public int value;
        AutoStartPos(int value)
        {
            this.value = value;
        }   //AutoStartPos
    }   //enum AutoStartPos

    public enum AutoStrategy
    {
        StartPos1Auto,
        PurePursuitDrive,
        PidDrive,
        TimedDrive,
        HybridModeAuto,
        DoNothing
    }   //enum AutoStrategy

    /**
     * This class encapsulates all user choices for autonomous mode from the smart dashboard.
     *
     * To add an autonomous choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add getter code to the fetchChoices method.
     * 5. Add an entry of the new choice to the toString method.
     */
    public static class AutoChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        // Choice menus
        private final FrcChoiceMenu<DriverStation.Alliance> allianceMenu;
        private final FrcChoiceMenu<AutoStartPos> autoStartPosMenu;
        private final FrcChoiceMenu<AutoStrategy> autoStrategyMenu;

        // Standard auto choices
        public Alliance alliance;
        public AutoStartPos startPos;
        public AutoStrategy strategy;
        public double startDelay;
        public double xDriveDistance;
        public double yDriveDistance;
        public double turnAngle;
        public double drivePower;
        public double driveTime;
        public String pathFile;
        // Game specific options

        public AutoChoices()
        {
            //
            // Create autonomous mode specific choice menus and populate them.
            //
            // Alliance menu.
            allianceMenu = new FrcChoiceMenu<>(DBKEY_ALLIANCE);
            allianceMenu.addChoice(DriverStation.Alliance.Red.name(), DriverStation.Alliance.Red);
            allianceMenu.addChoice(DriverStation.Alliance.Blue.name(), DriverStation.Alliance.Blue, true, true);
            userChoices.addChoiceMenu(DBKEY_ALLIANCE, allianceMenu);
            // StartPos menu.
            autoStartPosMenu = new FrcChoiceMenu<>(DBKEY_START_POS);
            autoStartPosMenu.addChoice(AutoStartPos.StartPos1.name(), AutoStartPos.StartPos1, true, false);
            autoStartPosMenu.addChoice(AutoStartPos.StartPos2.name(), AutoStartPos.StartPos2);
            autoStartPosMenu.addChoice(AutoStartPos.StartPos3.name(), AutoStartPos.StartPos3, false, true);
            userChoices.addChoiceMenu(DBKEY_START_POS, autoStartPosMenu);
            // Strategy menu.
            autoStrategyMenu = new FrcChoiceMenu<>(DBKEY_STRATEGY);
            if (RobotParams.Preferences.hybridMode)
            {
                autoStrategyMenu.addChoice(AutoStrategy.HybridModeAuto.name(), AutoStrategy.HybridModeAuto);
            }
            else
            {
                autoStrategyMenu.addChoice(AutoStrategy.StartPos1Auto.name(), AutoStrategy.StartPos1Auto);
                autoStrategyMenu.addChoice(AutoStrategy.PurePursuitDrive.name(), AutoStrategy.PurePursuitDrive);
                autoStrategyMenu.addChoice(AutoStrategy.PidDrive.name(), AutoStrategy.PidDrive);
                autoStrategyMenu.addChoice(AutoStrategy.TimedDrive.name(), AutoStrategy.TimedDrive);
            }
            autoStrategyMenu.addChoice(AutoStrategy.DoNothing.name(), AutoStrategy.DoNothing, true, true);
            userChoices.addChoiceMenu(DBKEY_STRATEGY, autoStrategyMenu);
            // Start delay.
            userChoices.addNumber(DBKEY_START_DELAY, 0.0);
            // Option choices for other Drive strategies.
            userChoices.addNumber(DBKEY_X_DRIVE_TARGET, 0.0);   // in feet
            userChoices.addNumber(DBKEY_Y_DRIVE_TARGET, 0.0);   // in feet
            userChoices.addNumber(DBKEY_TURN_TARGET, 0.0);      // in degrees
            userChoices.addNumber(DBKEY_DRIVE_POWER, 0.0);
            userChoices.addNumber(DBKEY_DRIVE_TIME, 0.0);       // in seconds
            userChoices.addString(DBKEY_PATHFILE, "DrivePath.csv");
            // Game specific options
        }   //AutoChoices

        public void fetchChoices()
        {
            // Get alliance info from FMS if one is connected. If not, get it from dashboard.
            FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
            alliance = matchInfo.eventName != null? matchInfo.alliance: allianceMenu.getCurrentChoiceObject();
            startPos = autoStartPosMenu.getCurrentChoiceObject();
            strategy = autoStrategyMenu.getCurrentChoiceObject();
            startDelay = userChoices.getUserNumber(DBKEY_START_DELAY);

            xDriveDistance = userChoices.getUserNumber(DBKEY_X_DRIVE_TARGET);
            yDriveDistance = userChoices.getUserNumber(DBKEY_Y_DRIVE_TARGET);
            turnAngle = userChoices.getUserNumber(DBKEY_TURN_TARGET);
            drivePower = userChoices.getUserNumber(DBKEY_DRIVE_POWER);
            driveTime = userChoices.getUserNumber(DBKEY_DRIVE_TIME);
            pathFile = userChoices.getUserString(DBKEY_PATHFILE);
            // Game specific options
        }   //fetchChoices

        @Override
        public String toString()
        {
            return "alliance=\"" + alliance + "\" " +
                   "startPos=\"" + startPos + "\" " +
                   "strategy=\"" + strategy + "\" " +
                   "startDelay=" + startDelay + " sec " +

                   "xDistance=" + xDriveDistance + " ft " +
                   "yDistance=" + yDriveDistance + " ft " +
                   "turnDegrees=" + turnAngle + " deg " +
                   "drivePower=" + drivePower + "\" " +
                   "driveTime=" + driveTime + " sec " +
                   "pathFile=\"" + pathFile + "\" ";
        }   //toString

    }   //class AutoChoices

    //
    // Global objects.
    //

    public static final AutoChoices autoChoices = new AutoChoices();
    private final Robot robot;
    private final TrcRobot.RobotCommand startPos1Auto;
    private TrcRobot.RobotCommand autoCommand = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcAuto(Robot robot)
    {
        //
        // Create and initialize global objects.
        //
        this.robot = robot;
        robot.dashboard.refreshKey(DBKEY_CHOICES_REFRESH, false);
        startPos1Auto = new CmdAuto(robot, autoChoices);
    }   //FrcAuto

    /**
     * This method checks if an autonomous command is running.
     *
     * @return true if autonomous command is running, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoCommand != null && autoCommand.isActive();
    }   //isAutoActive

    /**
     * This method cancels the autonomous command if one is running.
     */
    public void cancel()
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // This makes sure that the autonomous stops running when
            // teleop starts running. If you want the autonomous to
            // continue until interrupted by another command, remove
            // this line or comment it out.
            if (robot.m_autonomousCommand != null)
            {
                robot.m_autonomousCommand.cancel();
            }
        }

        if (autoCommand != null)
        {
            autoCommand.cancel();
            autoCommand = null;
        }
    }   //cancel

    //
    // Implements TrcRobot.RunMode.
    //

    /**
     * This method is called when the autonomous mode is about to start. Typically, you put code that will prepare
     * the robot for start of autonomous here such as creating autonomous command according to the chosen autonomous
     * strategy, initializing autonomous command and enabling/configuring sensors and subsystems that are necessary
     * for the autonomous command.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Create autonomous command.
        //
        switch (autoChoices.strategy)
        {
            case StartPos1Auto:
                if (robot.robotBase != null)
                {
                    autoCommand = startPos1Auto;
                }
                break;

            case PurePursuitDrive:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdPurePursuitDrive(
                        robot.robotBase.driveBase, robot.robotInfo.baseParams.xDrivePidCoeffs,
                        robot.robotInfo.baseParams.yDrivePidCoeffs, robot.robotInfo.baseParams.turnPidCoeffs,
                        robot.robotInfo.baseParams.velPidCoeffs);
                    ((CmdPurePursuitDrive) autoCommand).startPath(
                        0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        RobotParams.Robot.teamFolderPath + "/" + autoChoices.pathFile, false);
                }
                break;

            case PidDrive:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    ((CmdPidDrive) autoCommand).startPath(
                        autoChoices.startDelay, autoChoices.drivePower, null,
                        new TrcPose2D(autoChoices.xDriveDistance*12.0,
                                      autoChoices.yDriveDistance*12.0,
                                      autoChoices.turnAngle));
                }
                break;

            case TimedDrive:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, autoChoices.startDelay, autoChoices.driveTime, 0.0,
                        autoChoices.drivePower, 0.0);
                }
                break;

            case HybridModeAuto:
            case DoNothing:
            default:
                autoCommand = null;
                break;
        }
        robot.globalTracer.logInfo(moduleName, "MatchInfo", FrcMatchInfo.getMatchInfo().toString());
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());

        if (autoCommand != null)
        {
            autoCommand.start();
        }
    }   //startMode

    /**
     * This method is called when autonomous mode is about to end. Typically, you put code that will do clean
     * up here such as canceling unfinished autonomous command and disabling autonomous sensors and subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        cancel();
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
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //periodic

}   //class FrcAuto
