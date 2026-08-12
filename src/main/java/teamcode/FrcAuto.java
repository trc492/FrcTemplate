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
    //
    // Global constants.
    //

    //
    // Auto choices enums.
    //
    public enum AutoStrategy
    {
        TEMPLATE_AUTO,
        PP_DRIVE,
        PID_DRIVE,
        TIMED_DRIVE,
        HYBRID_MODE_AUTO,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum AutoStartPos
    {
        POS_1(0),
        POS_2(1),
        POS_3(2);
        // The value can be used as index into arrays if necessary.
        public int value;
        AutoStartPos(int value)
        {
            this.value = value;
        }   //AutoStartPos
    }   //enum AutoStartPos

    /**
     * This class encapsulates all user choices for autonomous mode from the smart dashboard.
     *
     * To add an autonomous choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    public static class AutoChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        // Choice menus
        private final FrcChoiceMenu<DriverStation.Alliance> allianceMenu;
        private final FrcChoiceMenu<AutoStrategy> autoStrategyMenu;
        private final FrcChoiceMenu<AutoStartPos> autoStartPosMenu;

        public Alliance alliance;
        public AutoStrategy strategy;
        public AutoStartPos startPos;
        public double startDelay;

        public String pathFile;
        public double xDriveDistance;
        public double yDriveDistance;
        public double turnAngle;
        public double driveTime;
        public double drivePower;

        public AutoChoices()
        {
            //
            // Create autonomous mode specific choice menus.
            //
            allianceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_ALLIANCE);
            autoStrategyMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_STRATEGY);
            autoStartPosMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_START_POS);
            //
            // Populate autonomous mode choice menus.
            //
            allianceMenu.addChoice("Red", DriverStation.Alliance.Red);
            allianceMenu.addChoice("Blue", DriverStation.Alliance.Blue, true, true);

            if (RobotParams.Preferences.hybridMode)
            {
                autoStrategyMenu.addChoice("Hybrid-mode Auto", AutoStrategy.HYBRID_MODE_AUTO);
            }
            else
            {
                autoStrategyMenu.addChoice("Template Auto", AutoStrategy.TEMPLATE_AUTO);
                autoStrategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PP_DRIVE);
                autoStrategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE);
                autoStrategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE);
            }
            autoStrategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING, true, true);

            autoStartPosMenu.addChoice("Start Position 1", AutoStartPos.POS_1, true, false);
            autoStartPosMenu.addChoice("Start Position 2", AutoStartPos.POS_2);
            autoStartPosMenu.addChoice("Start Position 3", AutoStartPos.POS_3, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_ALLIANCE, allianceMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_STRATEGY, autoStrategyMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_START_POS, autoStartPosMenu);
            userChoices.addNumber(Dashboard.DBKEY_AUTO_START_DELAY, 0.0);

            userChoices.addString(Dashboard.DBKEY_AUTO_PATHFILE, "DrivePath.csv");
            userChoices.addNumber(Dashboard.DBKEY_AUTO_X_DRIVE_DISTANCE, 0.0);      // in feet
            userChoices.addNumber(Dashboard.DBKEY_AUTO_Y_DRIVE_DISTANCE, 0.0);      // in feet
            userChoices.addNumber(Dashboard.DBKEY_AUTO_TURN_ANGLE, 0.0);            // in degrees
            userChoices.addNumber(Dashboard.DBKEY_AUTO_DRIVE_TIME, 0.0);            // in seconds
            userChoices.addNumber(Dashboard.DBKEY_AUTO_DRIVE_POWER, 0.0);
        }   //AutoChoices

        public void fetchChoices()
        {
            // Get alliance info from FMS if one is connected. If not, get it from dashboard.
            FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
            alliance = matchInfo.eventName != null? matchInfo.alliance: allianceMenu.getCurrentChoiceObject();
            strategy = autoStrategyMenu.getCurrentChoiceObject();
            startPos = autoStartPosMenu.getCurrentChoiceObject();
            startDelay = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_START_DELAY);

            pathFile = userChoices.getUserString(Dashboard.DBKEY_AUTO_PATHFILE);
            xDriveDistance = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_X_DRIVE_DISTANCE);
            yDriveDistance = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_Y_DRIVE_DISTANCE);
            turnAngle = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_TURN_ANGLE);
            driveTime = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_DRIVE_TIME);
            drivePower = userChoices.getUserNumber(Dashboard.DBKEY_AUTO_DRIVE_POWER);
        }   //fetchChoices

        @Override
        public String toString()
        {
            return "alliance=\"" + alliance + "\" " +
                   "strategy=\"" + strategy + "\" " +
                   "startPos=\"" + startPos + "\" " +
                   "startDelay=" + startDelay + " sec " +

                   "pathFile=\"" + pathFile + "\" " +
                   "xDistance=" + xDriveDistance + " ft " +
                   "yDistance=" + yDriveDistance + " ft " +
                   "turnDegrees=" + turnAngle + " deg " +
                   "driveTime=" + driveTime + " sec " +
                   "drivePower=" + drivePower + "\" ";
        }   //toString

    }   //class AutoChoices

    //
    // Global objects.
    //

    public static final AutoChoices autoChoices = new AutoChoices();
    private final Robot robot;
    private final TrcRobot.RobotCommand templateAuto;
    private TrcRobot.RobotCommand autoCommand;

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
        templateAuto = new CmdAuto(robot, autoChoices);
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
        // Retrieve Auto choices.
        //
        robot.globalTracer.logInfo(moduleName, "MatchInfo", FrcMatchInfo.getMatchInfo().toString());
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
        //
        // Create autonomous command.
        //
        switch (autoChoices.strategy)
        {
            case TEMPLATE_AUTO:
                if (robot.robotBase != null)
                {
                    autoCommand = templateAuto;
                }
                break;

            case PP_DRIVE:
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

            case PID_DRIVE:
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

            case TIMED_DRIVE:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, autoChoices.startDelay, autoChoices.driveTime, 0.0,
                        autoChoices.drivePower, 0.0);
                }
                break;

            case HYBRID_MODE_AUTO:
            case DO_NOTHING:
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
        //
        // Stop autonomous command.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
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
