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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.util.Arrays;
import java.util.Locale;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcUserChoices;
import frclib.driverio.FrcXboxController;
import frclib.vision.FrcPhotonVision;
import teamcode.vision.PhotonVision.PipelineType;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = FrcTest.class.getSimpleName();
    // Smart dashboard keys for Autonomous choices.
    private static final String DBKEY_TEST_TESTS = "Test/Tests";
    private static final String DBKEY_TEST_X_TARGET = "Test/XTarget";
    private static final String DBKEY_TEST_Y_TARGET = "Test/YTarget";
    private static final String DBKEY_TEST_TURN_TARGET = "Test/TurnTarget";
    private static final String DBKEY_TEST_DRIVE_POWER = "Test/DrivePower";
    private static final String DBKEY_TEST_TURN_POWER = "Test/TurnPower";
    private static final String DBKEY_TEST_DRIVE_TIME = "Test/DriveTime";
    private static final String DBKEY_TEST_X_KP = "Test/XKp";
    private static final String DBKEY_TEST_X_KI = "Test/XKi";
    private static final String DBKEY_TEST_X_KD = "Test/XKd";
    private static final String DBKEY_TEST_X_KF = "Test/XKf";
    private static final String DBKEY_TEST_X_IZONE = "Test/XIZone";
    private static final String DBKEY_TEST_Y_KP = "Test/YKp";
    private static final String DBKEY_TEST_Y_KI = "Test/YKi";
    private static final String DBKEY_TEST_Y_KD = "Test/YKd";
    private static final String DBKEY_TEST_Y_KF = "Test/YKf";
    private static final String DBKEY_TEST_Y_IZONE = "Test/YIZone";
    private static final String DBKEY_TEST_TURN_KP = "Test/TurnKp";
    private static final String DBKEY_TEST_TURN_KI = "Test/TurnKi";
    private static final String DBKEY_TEST_TURN_KD = "Test/TurnKd";
    private static final String DBKEY_TEST_TURN_KF = "Test/TurnKf";
    private static final String DBKEY_TEST_TURN_IZONE = "Test/TurnIZone";
    private static final String DBKEY_TEST_SUBSYSTEM_NAME = "Test/SubsystemName";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM0 = "Test/SubsystemParam0";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM1 = "Test/SubsystemParam1";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM2 = "Test/SubsystemParam2";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM3 = "Test/SubsystemParam3";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM4 = "Test/SubsystemParam4";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM5 = "Test/SubsystemParam5";
    public static final String DBKEY_TEST_SUBSYSTEM_PARAM6 = "Test/SubsystemParam6";
    private static final String DBKEY_TEST_MAX_VELOCITY = "Test/MaxVelocity";
    private static final String DBKEY_TEST_MAX_ACCELERATION = "Test/MaxAcceleration";
    private static final String DBKEY_TEST_MAX_DECELERATION = "Test/MaxDeceleration";
    private static final String DBKEY_TEST_ROBOT_VEL = "Test/RobotVelocity";
    private static final String DBKEY_TEST_TARGET_VEL = "Test/TargetVelocity";
    private static final String DBKEY_TEST_ROBOT_POS = "Test/RobotPosition";
    private static final String DBKEY_TEST_TARGET_POS = "Test/TargetPosition";
    //
    // Global constants.
    //

    //
    // Tests.
    //
    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        VISION_TEST,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_DRIVE_PID,
        TUNE_SUBSYSTEM,
        LIVE_WINDOW
    }   //enum Test

    /**
     * This class encapsulates all user choices for test mode from the smart dashboard.
     *
     * To add a test choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    class TestChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            //
            // Create test mode specific choice menus.
            //
            testMenu = new FrcChoiceMenu<>(DBKEY_TEST_TESTS);
            //
            // Populate test mode menus.
            //
            testMenu.addChoice("Sensors Test", Test.SENSORS_TEST, true, false);
            testMenu.addChoice("Subsystems Test", Test.SUBSYSTEMS_TEST);
            testMenu.addChoice("Vision Test", Test.VISION_TEST);
            testMenu.addChoice("Drive Speed Test", Test.DRIVE_SPEED_TEST);
            testMenu.addChoice("Drive Motors Test", Test.DRIVE_MOTORS_TEST);
            testMenu.addChoice("X Timed Drive", Test.X_TIMED_DRIVE);
            testMenu.addChoice("Y Timed Drive", Test.Y_TIMED_DRIVE);
            testMenu.addChoice("PurePursuit Drive", Test.PP_DRIVE);
            testMenu.addChoice("PID Drive", Test.PID_DRIVE);
            testMenu.addChoice("Tune Drive PID", Test.TUNE_DRIVE_PID);
            testMenu.addChoice("Tune Subsystem", Test.TUNE_SUBSYSTEM);
            testMenu.addChoice("Live Window", Test.LIVE_WINDOW, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(DBKEY_TEST_TESTS, testMenu);
            userChoices.addNumber(DBKEY_TEST_X_TARGET, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_Y_TARGET, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_TURN_TARGET, 0.0); // in degrees
            userChoices.addNumber(DBKEY_TEST_DRIVE_POWER, 0.5);
            userChoices.addNumber(DBKEY_TEST_TURN_POWER, 0.5);
            userChoices.addNumber(DBKEY_TEST_DRIVE_TIME, 0.0);  // in seconds
            userChoices.addNumber(DBKEY_TEST_X_KP, 0.0);
            userChoices.addNumber(DBKEY_TEST_X_KI, 0.0);
            userChoices.addNumber(DBKEY_TEST_X_KD, 0.0);
            userChoices.addNumber(DBKEY_TEST_X_KF, 0.0);
            userChoices.addNumber(DBKEY_TEST_X_IZONE, 0.0);
            userChoices.addNumber(DBKEY_TEST_Y_KP, 0.0);
            userChoices.addNumber(DBKEY_TEST_Y_KI, 0.0);
            userChoices.addNumber(DBKEY_TEST_Y_KD, 0.0);
            userChoices.addNumber(DBKEY_TEST_Y_KF, 0.0);
            userChoices.addNumber(DBKEY_TEST_Y_IZONE, 0.0);
            userChoices.addNumber(DBKEY_TEST_TURN_KP, 0.0);
            userChoices.addNumber(DBKEY_TEST_TURN_KI, 0.0);
            userChoices.addNumber(DBKEY_TEST_TURN_KD, 0.0);
            userChoices.addNumber(DBKEY_TEST_TURN_KF, 0.0);
            userChoices.addNumber(DBKEY_TEST_TURN_IZONE, 0.0);
            userChoices.addNumber(DBKEY_TEST_MAX_VELOCITY, 0.0);
            userChoices.addNumber(DBKEY_TEST_MAX_ACCELERATION, 0.0);
            userChoices.addNumber(DBKEY_TEST_MAX_DECELERATION, 0.0);
            userChoices.addNumber(DBKEY_TEST_ROBOT_VEL, 0.0);
            userChoices.addNumber(DBKEY_TEST_TARGET_VEL, 0.0);
            userChoices.addNumber(DBKEY_TEST_ROBOT_POS, 0.0);
            userChoices.addNumber(DBKEY_TEST_TARGET_POS, 0.0);
            userChoices.addString(DBKEY_TEST_SUBSYSTEM_NAME, "");
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM0, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM1, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM2, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM3, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM4, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM5, 0.0);
            userChoices.addNumber(DBKEY_TEST_SUBSYSTEM_PARAM6, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getXTarget()
        {
            return userChoices.getUserNumber(DBKEY_TEST_X_TARGET);
        }   //getXTarget

        public double getYTarget()
        {
            return userChoices.getUserNumber(DBKEY_TEST_Y_TARGET);
        }   //getYTarget

        public double getTurnTarget()
        {
            return userChoices.getUserNumber(DBKEY_TEST_TURN_TARGET);
        }   //getTurnTarget

        public double getDrivePower()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_POWER);
        }   //getDrivePower

        public double getTurnPower()
        {
            return userChoices.getUserNumber(DBKEY_TEST_TURN_POWER);
        }   //getTurnPower

        public double getDriveTime()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_TIME);
        }   //getDriveTime

        public TrcPidController.PidCoefficients getXPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TEST_X_KP),
                userChoices.getUserNumber(DBKEY_TEST_X_KI),
                userChoices.getUserNumber(DBKEY_TEST_X_KD),
                userChoices.getUserNumber(DBKEY_TEST_X_KF),
                userChoices.getUserNumber(DBKEY_TEST_X_IZONE));
        }   //getXPidCoefficients

        public TrcPidController.PidCoefficients getYPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TEST_Y_KP),
                userChoices.getUserNumber(DBKEY_TEST_Y_KI),
                userChoices.getUserNumber(DBKEY_TEST_Y_KD),
                userChoices.getUserNumber(DBKEY_TEST_Y_KF),
                userChoices.getUserNumber(DBKEY_TEST_Y_IZONE));
        }   //getYPidCoefficients

        public TrcPidController.PidCoefficients getTurnPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TEST_TURN_KP),
                userChoices.getUserNumber(DBKEY_TEST_TURN_KI),
                userChoices.getUserNumber(DBKEY_TEST_TURN_KD),
                userChoices.getUserNumber(DBKEY_TEST_TURN_KF),
                userChoices.getUserNumber(DBKEY_TEST_TURN_IZONE));
        }   //getYPidCoefficients

        public double getMaxVelocity()
        {
            return userChoices.getUserNumber(DBKEY_TEST_MAX_VELOCITY);
        }   //getMaxVelocity

        public double getMaxAcceleration()
        {
            return userChoices.getUserNumber(DBKEY_TEST_MAX_ACCELERATION);
        }   //getMaxAcceleration

        public double getMaxDeceleration()
        {
            return userChoices.getUserNumber(DBKEY_TEST_MAX_ACCELERATION);
        }   //getMaxDeceleration

        public String getSubsystemName()
        {
            return userChoices.getUserString(DBKEY_TEST_SUBSYSTEM_NAME);
        }   //getSubsystemName

        public double[] getSubsystemTuneParams()
        {
            return new double[]
                {
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM0),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM1),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM2),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM3),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM4),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM5),
                    userChoices.getUserNumber(DBKEY_TEST_SUBSYSTEM_PARAM6)
                };
        }   //getSubsystemTuneParams

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "Test=\"%s\" " +
                "xTarget=\"%.1f ft\" " +
                "yTarget=\"%.1f ft\" " +
                "turnTarget=\"%.0f deg\" " +
                "drivePower=\"%.1f\" " +
                "turnPower=\"%.1f\" " +
                "driveTime=\"%.0f sec\" " +
                "xPidCoeff=\"%s\" " +
                "yPidCoeff=\"%s\" " +
                "turnPidCoeff=\"%s\" " +
                "maxVelocity=\"%.1f\" " +
                "maxAcceleration=\"%.1f\" " +
                "maxDeceleration=\"%.1f\" " +
                "subsystemName=\"%s\" " +
                "subsystemTuneParams=\"%s\" ",
                getTest(), getXTarget(), getYTarget(), getTurnTarget(), getDrivePower(), getTurnPower(),
                getDriveTime(), getXPidCoefficients(), getYPidCoefficients(), getTurnPidCoefficients(),
                getMaxVelocity(), getMaxAcceleration(), getMaxDeceleration(), getSubsystemName(),
                Arrays.toString(getSubsystemTuneParams()));
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    private final TestChoices testChoices = new TestChoices();
    private TrcRobot.RobotCommand testCommand;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    // Tune Drive PID.
    private TrcPose2D tuneDriveStartPoint = null;
    private TrcPose2D tuneDriveEndPoint = null;
    private boolean tuneDriveAtEndPoint = false;
    // Vision Pipelines.
    private PipelineType frontPipeline = PipelineType.APRILTAG;
    private PipelineType backPipeline = PipelineType.APRILTAG;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);
        //
        // Create and initialize global objects.
        //

    }   //FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // Cancels all running commands at the start of test mode.
            CommandScheduler.getInstance().cancelAll();
        }
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        //
        // Retrieve Test choices.
        //
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);
        //
        // Create Command objects according to test choice.
        //
        boolean liveWindowEnabled = false;

        switch (testChoices.getTest())
        {
            case SENSORS_TEST:
                // Make sure no joystick controls on sensors test.
                setControlsEnabled(false);
                break;

            case VISION_TEST:
                if (robot.photonVisionFront != null)
                {
                    robot.photonVisionFront.setPipeline(frontPipeline);
                }

                if (robot.photonVisionBack != null)
                {
                    robot.photonVisionBack.setPipeline(backPipeline);
                }
                break;

            case DRIVE_MOTORS_TEST:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotDrive.driveBase, robot.robotDrive.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), testChoices.getDrivePower(),
                        0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), 0.0, testChoices.getDrivePower(),
                        0.0);
                }
                break;

            case PP_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                    robot.robotDrive.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                    robot.robotDrive.purePursuitDrive.start(
                        true,
                        robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxDeceleration,
                        new TrcPose2D(
                            testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                            testChoices.getTurnTarget()));
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.pidDrive != null)
                {
                    robot.robotDrive.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), null,
                        new TrcPose2D(
                            testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                            testChoices.getTurnTarget()));
                }
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // Start test state machine if necessary.
        //

    }   //startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        switch (testChoices.getTest())
        {
            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                break;

            default:
                break;
        }

        if (testCommand != null)
        {
            testCommand.cancel();
        }

        super.stopMode(prevMode, nextMode);
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //

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
        int lineNum = 1;

        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Run test Cmd.
        //
        switch (testChoices.getTest())
        {
            case DRIVE_SPEED_TEST:
                if (robot.robotDrive != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;
                    double deceleration = 0.0;
                    double deltaTime = currTime - prevTime;

                    if (prevTime != 0.0)
                    {
                        if (velocity > prevVelocity)
                        {
                            acceleration = (velocity - prevVelocity)/deltaTime;
                        }
                        else
                        {
                            deceleration = (prevVelocity - velocity)/deltaTime;
                        }
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    if (deceleration > maxDriveDeceleration)
                    {
                        maxDriveDeceleration = deceleration;
                    }

                    if (velPose.angle > maxTurnVelocity)
                    {
                        maxTurnVelocity = velPose.angle;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                }
                break;

            case TUNE_DRIVE_PID:
                if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                {
                    robot.dashboard.putNumber(
                        DBKEY_TEST_ROBOT_VEL, robot.robotDrive.purePursuitDrive.getPathRobotVelocity());
                    robot.dashboard.putNumber(
                        DBKEY_TEST_TARGET_VEL, robot.robotDrive.purePursuitDrive.getPathTargetVelocity());
                    robot.dashboard.putNumber(
                        DBKEY_TEST_ROBOT_POS, robot.robotDrive.purePursuitDrive.getPathRelativePosition());
                    robot.dashboard.putNumber(
                        DBKEY_TEST_TARGET_POS, robot.robotDrive.purePursuitDrive.getPathPositionTarget());
                }
                break;
    
            default:
                break;
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            //
            // Call super.runPeriodic only if you need TeleOp control of the robot.
            //
            switch (testChoices.getTest())
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                case VISION_TEST:
                    displaySensorStates(lineNum);
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotDrive != null)
                    {
                        double lfEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPosition());
                        double rfEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPosition());
                        double lbEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPosition(): 0.0);
                        double rbEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPosition(): 0.0);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:lf=%f,rf=%f", lfEnc, rfEnc);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:lb=%f,rb=%f", lbEnc, rbEnc);
                        robot.dashboard.displayPrintf(lineNum++, "EncAverage=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    }
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_DRIVE_PID:
                    if (robot.robotDrive != null)
                    {
                        TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                        if (robot.robotDrive.purePursuitDrive != null)
                        {
                            xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                        }
                        else if (testChoices.getTest() == Test.PID_DRIVE && robot.robotDrive.pidDrive != null)
                        {
                            xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (yPidCtrl != null)
                        {
                            yPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (turnPidCtrl != null)
                        {
                            turnPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                    }
                    break;

                default:
                    break;
            }
            //
            // Update Dashboard.
            //
            Dashboard.updateDashboard(robot, lineNum);
        }
    }   //periodic

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        Test test = testChoices.getTest();

        return test == Test.SUBSYSTEMS_TEST || test == Test.TUNE_SUBSYSTEM || test == Test.VISION_TEST ||
               test == Test.DRIVE_SPEED_TEST;
    }   //allowTeleOp

    //
    // Overriding ButtonEvent here if necessary.
    //
    /**
     * This method is called when an driver controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void driverControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
            case Back:
                break;

            case Start:
                Test test = testChoices.getTest();

                if (test == Test.TUNE_DRIVE_PID)
                {
                    if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                robot.robotDrive.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotDrive.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                                        testChoices.getTurnTarget()));
                                tuneDriveAtEndPoint = false;
                            }
                            robot.robotDrive.purePursuitDrive.setXPositionPidCoefficients(
                                testChoices.getXPidCoefficients());
                            robot.robotDrive.purePursuitDrive.setYPositionPidCoefficients(
                                testChoices.getYPidCoefficients());
                            robot.robotDrive.purePursuitDrive.setTurnPidCoefficients(
                                testChoices.getTurnPidCoefficients());
                            robot.robotDrive.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                            robot.robotDrive.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                            robot.robotDrive.purePursuitDrive.start(
                                false,
                                testChoices.getMaxVelocity(),
                                testChoices.getMaxAcceleration(),
                                testChoices.getMaxDeceleration(),
                                tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        passToTeleOp = false;
                    }
                }
                else if (test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        String subsystemName = testChoices.getSubsystemName();
                        robot.globalTracer.traceInfo(moduleName, "Tune subsystem: " + subsystemName);
                        if (subsystemName.isEmpty())
                        {
                            TrcSubsystem subsystem = TrcSubsystem.getSubsystem(subsystemName);

                            if (subsystem != null)
                            {
                                subsystem.prepSubsystemForTuning(testChoices.getSubsystemTuneParams());
                            }
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            default:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.driverControllerButtonEvent(button, pressed);
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
            case Back:
            case Start:
            default:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorControllerButtonEvent(button, pressed);
        }
    }   //operatorControllerButtonEvent

    //
    // Implement tests.
    //
    /**
     * This method reads all sensors and prints out their values. This is a very
     * useful diagnostic tool to check if all sensors are working properly. For
     * encoders, since test subsystem mode is also teleop mode, you can operate
     * the joysticks to turn the motors and check the corresponding encoder
     * counts.
     *
     * @param lineNum specifies the dashboard starting line number to display the info.
     */
    private void displaySensorStates(int lineNum)
    {
        //
        // Display drivebase info.
        //
        if (robot.battery != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "Sensors Test (Batt=%.1f/%.1f):", robot.battery.getVoltage(), robot.battery.getLowestVoltage());
        }

        if (robot.robotDrive != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "DriveBase: Pose=%s,Vel=%s", robot.robotDrive.driveBase.getFieldPosition(),
                robot.robotDrive.driveBase.getFieldVelocity());
            robot.dashboard.displayPrintf(lineNum++, "DrivePower: lf=%.2f,rf=%.2f,lb=%.2f,rb=%.2f",
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPower(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPower(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPower(): 0.0,
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPower(): 0.0);
            robot.dashboard.displayPrintf(lineNum++, "DriveEnc: lf=%.1f,rf=%.1f,lb=%.1f,rb=%.1f",
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0,
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0);
            if (robot.robotDrive instanceof FrcSwerveDrive)
            {
                FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robot.robotDrive;
                robot.dashboard.displayPrintf(lineNum++, "SteerEnc: lf=%f, rf=%f, lb=%f, rb=%f",
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_BACK].getRawPosition());
            }
        }

        if (robot.photonVisionFront != null)
        {
            FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getBestDetectedObject(null);
            if (object != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "PhotonFront: pipeline=%s, pose=%s", frontPipeline, object.targetPose);
            }
            else
            {
                lineNum++;
            }
        }

        if (robot.photonVisionBack != null)
        {
            FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject(null);
            if (object != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "PhotonBack: pipeline=%s, pose=%s", backPipeline, object.targetPose);
            }
            else
            {
                lineNum++;
            }
        }
        //
        // Display other subsystems and sensor info.
        //

    }   //displaySensorStates

}   //class FrcTest
