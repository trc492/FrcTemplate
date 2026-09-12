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
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcMatchInfo;
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
    private static final String DBKEY_PREFIX            = "Auto/";
    private static final String DBKEY_ALLIANCE          = DBKEY_PREFIX + "Alliance";                //Choices
    private static final String DBKEY_START_POS         = DBKEY_PREFIX + "StartPos";                //Choices
    private static final String DBKEY_STRATEGY          = DBKEY_PREFIX + "Strategy";                //Choices
    private static final String DBKEY_START_DELAY       = DBKEY_PREFIX + "StartDelay";              //Number
    private static final String DBKEY_PP_DRIVE_PATH     = DBKEY_PREFIX + "PurePursuitDrivePath";    //Choices 
    private static final String DBKEY_X_DRIVE_TARGET    = DBKEY_PREFIX + "XDriveTarget";            //Number
    private static final String DBKEY_Y_DRIVE_TARGET    = DBKEY_PREFIX + "YDriveTarget";            //Number
    private static final String DBKEY_TURN_TARGET       = DBKEY_PREFIX + "TurnTarget";              //Number
    private static final String DBKEY_DRIVE_POWER       = DBKEY_PREFIX + "DrivePower";              //Number
    private static final String DBKEY_TURN_POWER        = DBKEY_PREFIX + "TurnPower";               //Number
    private static final String DBKEY_TIMED_DRIVE_POWER = DBKEY_PREFIX + "TimedDrivePower";         //Number
    private static final String DBKEY_TIMED_DRIVE_TIME  = DBKEY_PREFIX + "TimedDriveTime";          //Number
    private static final String DBKEY_PP_PATHFILE       = DBKEY_PREFIX + "PurePursuitPathFile";     //String
    // Game specific choices.

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

    public enum PurePursuitDrivePath
    {
        Path1,
        Path2
    }   //enum PurePursuitDrivePath

    /**
     * This class stores the autonomous menu choices.
     */
    public static class AutoChoices
    {
        private final FrcDashboard dashboard;
        // Choice menus
        private final FrcChoiceMenu<DriverStation.Alliance> allianceMenu;
        private final FrcChoiceMenu<AutoStartPos> startPosMenu;
        private final FrcChoiceMenu<AutoStrategy> strategyMenu;
        private final FrcChoiceMenu<PurePursuitDrivePath> ppDrivePathMenu;
        // Standard auto choices.
        public Alliance alliance;
        public AutoStartPos startPos;
        public AutoStrategy strategy;
        public double startDelay;
        public double xDriveDistance;
        public double yDriveDistance;
        public double turnAngle;
        public double drivePower;
        public double turnPower;
        public double timedDrivePower;
        public double timedDriveTime;
        public String purePursuitPathFile;
        // Game specific choices.

        public AutoChoices()
        {
            this.dashboard = FrcDashboard.getInstance();
            //
            // Create choice menus.
            //
            allianceMenu = new FrcChoiceMenu<>(DBKEY_ALLIANCE);
            startPosMenu = new FrcChoiceMenu<>(DBKEY_START_POS);
            strategyMenu = new FrcChoiceMenu<>(DBKEY_STRATEGY);
            ppDrivePathMenu = new FrcChoiceMenu<>(DBKEY_PP_DRIVE_PATH);
            //
            // Populate choice menus.
            //
            allianceMenu.addChoice(DriverStation.Alliance.Red.name(), DriverStation.Alliance.Red);
            allianceMenu.addChoice(DriverStation.Alliance.Blue.name(), DriverStation.Alliance.Blue, true, true);

            startPosMenu.addChoice(AutoStartPos.StartPos1.name(), AutoStartPos.StartPos1, true, false);
            startPosMenu.addChoice(AutoStartPos.StartPos2.name(), AutoStartPos.StartPos2);
            startPosMenu.addChoice(AutoStartPos.StartPos3.name(), AutoStartPos.StartPos3, false, true);

            if (RobotParams.Preferences.hybridMode)
            {
                strategyMenu.addChoice(AutoStrategy.HybridModeAuto.name(), AutoStrategy.HybridModeAuto);
            }
            else
            {
                strategyMenu.addChoice(AutoStrategy.StartPos1Auto.name(), AutoStrategy.StartPos1Auto);
                strategyMenu.addChoice(AutoStrategy.PurePursuitDrive.name(), AutoStrategy.PurePursuitDrive);
                strategyMenu.addChoice(AutoStrategy.PidDrive.name(), AutoStrategy.PidDrive);
                strategyMenu.addChoice(AutoStrategy.TimedDrive.name(), AutoStrategy.TimedDrive);
            }
            strategyMenu.addChoice(AutoStrategy.DoNothing.name(), AutoStrategy.DoNothing, true, true);

            ppDrivePathMenu.addChoice(PurePursuitDrivePath.Path1.name(), PurePursuitDrivePath.Path1, true, false);
            ppDrivePathMenu.addChoice(PurePursuitDrivePath.Path2.name(), PurePursuitDrivePath.Path2, false, true);
            //
            // Publish to Dashboard (Choice menus are Choosers and don't need publishing).
            //
            dashboard.refreshKey(DBKEY_START_DELAY, 0.0);
            dashboard.refreshKey(DBKEY_X_DRIVE_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_Y_DRIVE_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_TURN_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_POWER, 0.0);
            dashboard.refreshKey(DBKEY_TURN_POWER, 0.0);
            dashboard.refreshKey(DBKEY_TIMED_DRIVE_POWER, 0.0);
            dashboard.refreshKey(DBKEY_TIMED_DRIVE_TIME, 0.0);
            dashboard.refreshKey(DBKEY_PP_PATHFILE, "");
            // Game specific choices.
        }   //AutoChoices

        /**
         * This method fetches all choices from the Dashboard.
         */
        public void fetchChoices()
        {
            // Get alliance info from FMS if one is connected. If not, get it from dashboard.
            FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
            alliance = matchInfo.eventName != null? matchInfo.alliance: allianceMenu.getCurrentChoiceObject();

            startPos = startPosMenu.getCurrentChoiceObject();
            strategy = strategyMenu.getCurrentChoiceObject();
            startDelay = dashboard.getNumber(DBKEY_START_DELAY, 0.0);

            xDriveDistance = dashboard.getNumber(DBKEY_X_DRIVE_TARGET, 0.0);
            yDriveDistance = dashboard.getNumber(DBKEY_Y_DRIVE_TARGET, 0.0);
            turnAngle = dashboard.getNumber(DBKEY_TURN_TARGET, 0.0);
            drivePower = dashboard.getNumber(DBKEY_DRIVE_POWER, 0.0);
            turnPower = dashboard.getNumber(DBKEY_TURN_POWER, 0.0);
            timedDrivePower = dashboard.getNumber(DBKEY_TIMED_DRIVE_POWER, 0.0);
            timedDriveTime = dashboard.getNumber(DBKEY_TIMED_DRIVE_TIME, 0.0);
            purePursuitPathFile = dashboard.getString(DBKEY_PP_PATHFILE, "");
            // Game specific choices.
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
                   "turnPower=" + turnPower + "\" " +
                   "timedDrivePower=" + timedDrivePower + "\" " +
                   "timedDriveTime=" + timedDriveTime + " sec " +
                   "ppPathFile=\"" + purePursuitPathFile + "\" ";
                   // Game specific choices.
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

    /**
     * This method checks if an autonomous command is running.
     *
     * @return true if autonomous command is running, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoCommand != null && autoCommand.isActive();
    }   //isAutoActive

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
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
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
                        autoChoices.drivePower, autoChoices.turnPower,
                        RobotParams.Robot.teamFolderPath + "/" + autoChoices.purePursuitPathFile, false);
                }
                break;

            case PidDrive:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    autoCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    ((CmdPidDrive) autoCommand).startPath(
                        autoChoices.startDelay, autoChoices.drivePower, autoChoices.turnPower, null,
                        new TrcPose2D(autoChoices.xDriveDistance*12.0,
                                      autoChoices.yDriveDistance*12.0,
                                      autoChoices.turnAngle));
                }
                break;

            case TimedDrive:
                if (robot.robotBase != null)
                {
                    // TimedDrive only goes in the Y direction. Set up the robot to aim where you want to go.
                    autoCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, autoChoices.startDelay, autoChoices.timedDriveTime, 0.0,
                        autoChoices.timedDrivePower, 0.0);
                }
                break;

            case HybridModeAuto:
            case DoNothing:
            default:
                autoCommand = null;
                break;
        }

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
