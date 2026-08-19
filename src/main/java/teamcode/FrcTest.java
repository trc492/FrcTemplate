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

import java.util.Locale;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclib.drivebase.FrcSwerveBase;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcUserChoices;
import frclib.driverio.FrcXboxController;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.subsystem.TrcShooter.AimInfo;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = FrcTest.class.getSimpleName();

    // Test choices.
    private static final String DBKEY_PREFIX                = "Test/";
    private static final String DBKEY_TESTS                 = DBKEY_PREFIX + "Tests";                   //Choices
    // Drive Base tuning params.
    private static final String DBKEY_TUNE_X_TARGET         = DBKEY_PREFIX + "XTarget";                 //Number
    private static final String DBKEY_TUNE_Y_TARGET         = DBKEY_PREFIX + "YTarget";                 //Number
    private static final String DBKEY_TUNE_TURN_TARGET      = DBKEY_PREFIX + "TurnTarget";              //Number
    private static final String DBKEY_DRIVE_POWER           = DBKEY_PREFIX + "DrivePower";              //Number
    private static final String DBKEY_TURN_POWER            = DBKEY_PREFIX + "TurnPower";               //Number
    private static final String DBKEY_DRIVE_TIME            = DBKEY_PREFIX + "DriveTime";               //Number
    private static final String DBKEY_TUNE_KP               = DBKEY_PREFIX + "TuneKp";                  //Number
    private static final String DBKEY_TUNE_KI               = DBKEY_PREFIX + "TuneKi";                  //Number
    private static final String DBKEY_TUNE_KD               = DBKEY_PREFIX + "TuneKd";                  //Number
    private static final String DBKEY_TUNE_KF               = DBKEY_PREFIX + "TuneKf";                  //Number
    private static final String DBKEY_TUNE_IZONE            = DBKEY_PREFIX + "TuneIZone";               //Number
    private static final String DBKEY_MAX_VELOCITY          = DBKEY_PREFIX + "MaxVelocity";             //Number
    private static final String DBKEY_MAX_ACCELERATION      = DBKEY_PREFIX + "MaxAcceleration";         //Number
    private static final String DBKEY_MAX_DECELERATION      = DBKEY_PREFIX + "MaxDeceleration";         //Number
    private static final String DBKEY_ROBOT_VEL             = DBKEY_PREFIX + "RobotVelocity";           //Number
    private static final String DBKEY_TARGET_VEL            = DBKEY_PREFIX + "TargetVelocity";          //Number
    private static final String DBKEY_ROBOT_POS             = DBKEY_PREFIX + "RobotPosition";           //Number
    private static final String DBKEY_TARGET_POS            = DBKEY_PREFIX + "TargetPosition";          //Number
    // Subsystem tuning params.
    private static final String DBKEY_SUBSYSTEM_NAME        = DBKEY_PREFIX + "SubsystemName";           //String
    private static final String DBKEY_SUBSYSTEM_KP          = DBKEY_PREFIX + "SubsystemKp";             //Number
    private static final String DBKEY_SUBSYSTEM_KI          = DBKEY_PREFIX + "SubsystemKi";             //Number
    private static final String DBKEY_SUBSYSTEM_KD          = DBKEY_PREFIX + "SubsystemKd";             //Number
    private static final String DBKEY_SUBSYSTEM_KF          = DBKEY_PREFIX + "SubsystemKf";             //Number
    private static final String DBKEY_SUBSYSTEM_IZONE       = DBKEY_PREFIX + "SubsystemIZone";          //Number
    private static final String DBKEY_SUBSYSTEM_TOLERANCE   = DBKEY_PREFIX + "SubsystemTolerance";      //Number
    private static final String DBKEY_SUBSYSTEM_SOFTWARE_PID= DBKEY_PREFIX + "SubsystemSoftwarePid";    //Boolean
    private static final String DBKEY_SUBSYSTEM_KS          = DBKEY_PREFIX + "SubsystemKs";             //Number
    private static final String DBKEY_SUBSYSTEM_KV          = DBKEY_PREFIX + "SubsystemKv";             //Number
    private static final String DBKEY_SUBSYSTEM_KA          = DBKEY_PREFIX + "SubsystemKa";             //Number
    public static final String DBKEY_SUBSYSTEM_GRAVITY_POWER= DBKEY_PREFIX + "SubsystemGravityPower";   //Number
    public static final String DBKEY_SUBSYSTEM_TUNE_INPUT   = DBKEY_PREFIX + "TuneInput";               //Number
    public static final String DBKEY_SUBSYSTEM_TUNE_TARGET  = DBKEY_PREFIX + "TuneTarget";              //Number
    // Shoot Table tuning.
    private static final String DBKEY_SHOOT_DISTANCE        = DBKEY_PREFIX + "ShootDistance";           //Number
    private static final String DBKEY_SHOOT_VELOCITY        = DBKEY_PREFIX + "ShootVelocity";           //Number
    private static final String DBKEY_SHOOT_PAN_POS         = DBKEY_PREFIX + "ShootPanPos";             //Number
    private static final String DBKEY_SHOOT_TILT_POS        = DBKEY_PREFIX + "ShootTiltPos";            //Number
    //
    // Tests.
    //
    private enum Test
    {
        SubsystemsTest,
        DriveSpeedTest,
        DriveMotorsTest,
        XTimedDrive,
        YTimedDrive,
        PurePursuitDrive,
        PidDrive,
        TuneDriveXPid,
        TuneDriveYPid,
        TuneTurnPid,
        TuneSubsystem,
        TuneShootTable,
        VisionTest,
        SwerveCalibration,
        LiveWindow
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
    public static class TestChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            //
            // Create test mode specific choice menus.
            //
            testMenu = new FrcChoiceMenu<>(DBKEY_TESTS);
            //
            // Populate test mode menus.
            //
            testMenu.addChoice(Test.SubsystemsTest.name(), Test.SubsystemsTest, true, false);
            testMenu.addChoice(Test.DriveSpeedTest.name(), Test.DriveSpeedTest);
            testMenu.addChoice(Test.DriveMotorsTest.name(), Test.DriveMotorsTest);
            testMenu.addChoice(Test.XTimedDrive.name(), Test.XTimedDrive);
            testMenu.addChoice(Test.YTimedDrive.name(), Test.YTimedDrive);
            testMenu.addChoice(Test.PurePursuitDrive.name(), Test.PurePursuitDrive);
            testMenu.addChoice(Test.PidDrive.name(), Test.PidDrive);
            testMenu.addChoice(Test.TuneDriveXPid.name(), Test.TuneDriveXPid);
            testMenu.addChoice(Test.TuneDriveYPid.name(), Test.TuneDriveYPid);
            testMenu.addChoice(Test.TuneTurnPid.name(), Test.TuneTurnPid);
            testMenu.addChoice(Test.TuneSubsystem.name(), Test.TuneSubsystem);
            testMenu.addChoice(Test.TuneShootTable.name(), Test.TuneShootTable);
            testMenu.addChoice(Test.VisionTest.name(), Test.VisionTest);
            testMenu.addChoice(Test.SwerveCalibration.name(), Test.SwerveCalibration);
            testMenu.addChoice(Test.LiveWindow.name(), Test.LiveWindow, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(DBKEY_TESTS, testMenu);
            userChoices.addNumber(DBKEY_TUNE_X_TARGET, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TUNE_Y_TARGET, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TUNE_TURN_TARGET, 0.0); // in degrees
            userChoices.addNumber(DBKEY_DRIVE_POWER, 0.5);
            userChoices.addNumber(DBKEY_TURN_POWER, 0.5);
            userChoices.addNumber(DBKEY_DRIVE_TIME, 0.0);       // in seconds
            userChoices.addNumber(DBKEY_TUNE_KP, 0.0);
            userChoices.addNumber(DBKEY_TUNE_KI, 0.0);
            userChoices.addNumber(DBKEY_TUNE_KD, 0.0);
            userChoices.addNumber(DBKEY_TUNE_KF, 0.0);
            userChoices.addNumber(DBKEY_TUNE_IZONE, 0.0);
            userChoices.addNumber(DBKEY_MAX_VELOCITY, 0.0);
            userChoices.addNumber(DBKEY_MAX_ACCELERATION, 0.0);
            userChoices.addNumber(DBKEY_MAX_DECELERATION, 0.0);
            userChoices.addNumber(DBKEY_ROBOT_VEL, 0.0);
            userChoices.addNumber(DBKEY_TARGET_VEL, 0.0);
            userChoices.addNumber(DBKEY_ROBOT_POS, 0.0);
            userChoices.addNumber(DBKEY_TARGET_POS, 0.0);

            userChoices.addString(
                DBKEY_SUBSYSTEM_NAME,
                RobotParams.Preferences.testSubsystemName != null? RobotParams.Preferences.testSubsystemName: "");
            userChoices.addNumber(DBKEY_SUBSYSTEM_KP, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KI, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KD, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KF, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_IZONE, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_TOLERANCE, 0.0);
            userChoices.addBoolean(DBKEY_SUBSYSTEM_SOFTWARE_PID, false);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KS, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KV, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_KA, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_GRAVITY_POWER, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_TUNE_INPUT, 0.0);
            userChoices.addNumber(DBKEY_SUBSYSTEM_TUNE_TARGET, 0.0);
            // Shoot Table tuning.
            userChoices.addNumber(DBKEY_SHOOT_DISTANCE, 0.0);
            userChoices.addNumber(DBKEY_SHOOT_VELOCITY, 0.0);
            userChoices.addNumber(DBKEY_SHOOT_PAN_POS, 0.0);
            userChoices.addNumber(DBKEY_SHOOT_TILT_POS, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getTuneXTarget()
        {
            return userChoices.getUserNumber(DBKEY_TUNE_X_TARGET);
        }   //getTuneXTarget

        public double getTuneYTarget()
        {
            return userChoices.getUserNumber(DBKEY_TUNE_Y_TARGET);
        }   //getTuneYTarget

        public double getTuneTurnTarget()
        {
            return userChoices.getUserNumber(DBKEY_TUNE_TURN_TARGET);
        }   //getTuneTurnTarget

        public double getDrivePower()
        {
            return userChoices.getUserNumber(DBKEY_DRIVE_POWER);
        }   //getDrivePower

        public double getTurnPower()
        {
            return userChoices.getUserNumber(DBKEY_TURN_POWER);
        }   //getTurnPower

        public double getDriveTime()
        {
            return userChoices.getUserNumber(DBKEY_DRIVE_TIME);
        }   //getDriveTime

        public TrcPidController.PidCoefficients getTunePidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TUNE_KP),
                userChoices.getUserNumber(DBKEY_TUNE_KI),
                userChoices.getUserNumber(DBKEY_TUNE_KD),
                userChoices.getUserNumber(DBKEY_TUNE_KF),
                userChoices.getUserNumber(DBKEY_TUNE_IZONE));
        }   //getTunePidCoefficients

        public void setTunePidCoefficients(TrcPidController.PidCoefficients pidCoeffs)
        {
            userChoices.setUserNumber(DBKEY_TUNE_KP, pidCoeffs.kP);
            userChoices.setUserNumber(DBKEY_TUNE_KI, pidCoeffs.kI);
            userChoices.setUserNumber(DBKEY_TUNE_KD, pidCoeffs.kD);
            userChoices.setUserNumber(DBKEY_TUNE_KF, pidCoeffs.kF);
            userChoices.setUserNumber(DBKEY_TUNE_IZONE, pidCoeffs.iZone);
        }   //setTunePidCoefficients

        public double getMaxVelocity()
        {
            return userChoices.getUserNumber(DBKEY_MAX_VELOCITY);
        }   //getMaxVelocity

        public double getMaxAcceleration()
        {
            return userChoices.getUserNumber(DBKEY_MAX_ACCELERATION);
        }   //getMaxAcceleration

        public double getMaxDeceleration()
        {
            return userChoices.getUserNumber(DBKEY_MAX_DECELERATION);
        }   //getMaxDeceleration

        public String getSubsystemName()
        {
            return userChoices.getUserString(DBKEY_SUBSYSTEM_NAME);
        }   //getSubsystemName

        public TrcMotor.PidParams getSubsystemPidParameters()
        {
            return new TrcMotor.PidParams()
                .setPidCoefficients(
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KP),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KI),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KD),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KF),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_IZONE))
                .setFFCoefficients(
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KS),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KV),
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_KA))
                .setPidControlParams(
                    userChoices.getUserNumber(DBKEY_SUBSYSTEM_TOLERANCE),
                    userChoices.getUserBoolean(DBKEY_SUBSYSTEM_SOFTWARE_PID));
        }   //getSubsystemPidParameters

        public void setSubsystemPidParameters(TrcMotor.PidParams pidParams)
        {
            if (pidParams.pidCoeffs != null)
            {
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KP, pidParams.pidCoeffs.kP);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KI, pidParams.pidCoeffs.kI);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KD, pidParams.pidCoeffs.kD);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KF, pidParams.pidCoeffs.kF);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_IZONE, pidParams.pidCoeffs.iZone);
            }

            if (pidParams.ffCoeffs != null)
            {
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KS, pidParams.ffCoeffs.kS);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KV, pidParams.ffCoeffs.kV);
                userChoices.setUserNumber(DBKEY_SUBSYSTEM_KA, pidParams.ffCoeffs.kA);
            }

            userChoices.setUserNumber(DBKEY_SUBSYSTEM_TOLERANCE, pidParams.pidTolerance);
            userChoices.setUserBoolean(DBKEY_SUBSYSTEM_SOFTWARE_PID, pidParams.useSoftwarePid);
        }   //setSubsystemPidParameters

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
                "tunePidCoeff=\"%s\" " +
                "maxVelocity=\"%.1f\" " +
                "maxAcceleration=\"%.1f\" " +
                "maxDeceleration=\"%.1f\" " +
                "subsystemName=\"%s\" " +
                "subsystemPidParams=\"%s\" ",
                getTest(), getTuneXTarget(), getTuneYTarget(), getTuneTurnTarget(), getDrivePower(), getTurnPower(),
                getDriveTime(), getTunePidCoefficients(), getMaxVelocity(), getMaxAcceleration(), getMaxDeceleration(),
                getSubsystemName(), getSubsystemPidParameters());
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    public static final TestChoices testChoices = new TestChoices();
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
        Test test = testChoices.getTest();

        switch (test)
        {
            case DriveMotorsTest:
                if (robot.robotBase != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotBase.driveBase, robot.robotBase.driveMotors, 5.0, 0.5);
                }
                break;

            case XTimedDrive:
            case YTimedDrive:
                if (robot.robotBase != null &&
                    (test == Test.YTimedDrive || robot.robotBase.driveBase.supportsHolonomicDrive()))
                {
                    double xPower, yPower;

                    xPower = yPower = testChoices.getDrivePower();
                    if (test == Test.XTimedDrive)
                    {
                        yPower = 0.0;
                    }
                    else
                    {
                        xPower = 0.0;
                    }
                    robot.robotBase.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0, testChoices.getDriveTime(), xPower, yPower, 0.0);
                }
                break;

            case PurePursuitDrive:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                    robot.robotBase.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                    robot.robotBase.purePursuitDrive.start(
                        true, null,
                        new TrcPose2D(
                            testChoices.getTuneXTarget()*12.0, testChoices.getTuneYTarget()*12.0,
                            testChoices.getTuneTurnTarget()));
                }
                break;

            case PidDrive:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    ((CmdPidDrive) testCommand).startPath(
                        0.0, testChoices.getDrivePower(), null,
                        new TrcPose2D(
                            testChoices.getTuneXTarget()*12.0, testChoices.getTuneYTarget()*12.0,
                            testChoices.getTuneTurnTarget()));
                }
                break;

            case TuneDriveXPid:
            case TuneDriveYPid:
            case TuneTurnPid:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    TrcPidController.PidCoefficients tunePidCoeffs;

                    if (test == Test.TuneDriveXPid && robot.robotBase.purePursuitDrive.getXPosPidCtrl() != null)
                    {
                        tunePidCoeffs = robot.robotBase.purePursuitDrive.getXPosPidCtrl().getPidCoefficients();
                    }
                    else if (test == Test.TuneDriveYPid)
                    {
                        tunePidCoeffs = robot.robotBase.purePursuitDrive.getYPosPidCtrl().getPidCoefficients();
                    }
                    else
                    {
                        tunePidCoeffs = robot.robotBase.purePursuitDrive.getTurnPidCtrl().getPidCoefficients();
                    }

                    testChoices.setTunePidCoefficients(tunePidCoeffs);
                    robot.globalTracer.traceInfo(moduleName, "[%s]: TunePidCoeffs=%s.", test, tunePidCoeffs);
                }
                break;

            case VisionTest:
                if (robot.vision != null)
                {
                    robot.vision.setCam1Pipeline();
                    robot.vision.setCam2Pipeline();
                }
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Start Swerve Calibration.");
                    setControlsEnabled(false);
                    ((FrcSwerveBase) robot.robotBase).startSteeringCalibration();
                }
                break;

            case LiveWindow:
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
            case XTimedDrive:
            case YTimedDrive:
                // Cancel GyroAssist in case we turned it on for timed drive.
                robot.robotBase.driveBase.setGyroAssistEnabled(null);
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Stop Swerve Calibration.");
                    ((FrcSwerveBase) robot.robotBase).stopSteeringCalibration();
                }
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
        Test test = testChoices.getTest();

        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Run test Cmd.
        //
        switch (test)
        {
            case DriveSpeedTest:
                if (robot.robotBase != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotBase.driveBase.getRobotVelocity();
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

            case TuneDriveXPid:
            case TuneDriveYPid:
            case TuneTurnPid:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.dashboard.putNumber(
                        DBKEY_ROBOT_VEL, robot.robotBase.purePursuitDrive.getPathRobotVelocity());
                    robot.dashboard.putNumber(
                        DBKEY_TARGET_VEL, robot.robotBase.purePursuitDrive.getPathTargetVelocity());
                    robot.dashboard.putNumber(
                        DBKEY_ROBOT_POS, robot.robotBase.purePursuitDrive.getPathRelativePosition());
                    robot.dashboard.putNumber(
                        DBKEY_TARGET_POS, robot.robotBase.purePursuitDrive.getPathPositionTarget());
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
            switch (test)
            {
                case XTimedDrive:
                case YTimedDrive:
                    if (robot.robotBase != null)
                    {
                        double flEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.FrontLeft.value].getMotorPosition());
                        double frEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getMotorPosition());
                        double blEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.BackLeft.value] != null?
                                robot.robotBase.driveMotors[MotorIndex.BackLeft.value].getMotorPosition(): 0.0);
                        double brEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.BackRight.value] != null?
                                robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getMotorPosition(): 0.0);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:fl=%f,fr=%f", flEnc, frEnc);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:bl=%f,br=%f", blEnc, brEnc);
                        robot.dashboard.displayPrintf(lineNum++, "EncAverage=%f", (flEnc + frEnc + blEnc + brEnc) / 4.0);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                    }
                    break;

                case PurePursuitDrive:
                case PidDrive:
                case TuneDriveXPid:
                case TuneDriveYPid:
                case TuneTurnPid:
                    if (robot.robotBase != null)
                    {
                        TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                        if (robot.robotBase.purePursuitDrive != null)
                        {
                            xPidCtrl = robot.robotBase.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotBase.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotBase.purePursuitDrive.getTurnPidCtrl();
                        }
                        else if (test == Test.PidDrive && robot.robotBase.pidDrive != null)
                        {
                            xPidCtrl = robot.robotBase.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotBase.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotBase.pidDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
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

                case TuneShootTable:
                    if (robot.vision != null)
                    {
                        TrcPose2D targetPose = robot.getShooterToTargetPose();
                        if (targetPose != null)
                        {
                            robot.dashboard.putNumber(
                                DBKEY_SHOOT_DISTANCE, Math.hypot(targetPose.x, targetPose.y));
                        }
                    }
                    break;

                case VisionTest:
                    lineNum = doVisionTest(lineNum);
                    break;

                case SwerveCalibration:
                    if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                    {
                        FrcSwerveBase swerveBase = (FrcSwerveBase) robot.robotBase;
                        swerveBase.runSteeringCalibration();
                        swerveBase.displaySteerZeroCalibration(lineNum);
                    }
                    break;

                default:
                    break;
            }
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

        return test == Test.SubsystemsTest || test == Test.TuneSubsystem || test == Test.TuneShootTable ||
               test == Test.VisionTest || test == Test.DriveSpeedTest;
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
        Test test = testChoices.getTest();
        String subsystemName = null;

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));
        switch (button)
        {
            case A:
                if (test == Test.TuneShootTable && robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        if (robot.shooter.isActive())
                        {
                            if (robot.shooter.shooterMotor1.isVelocityOnTarget())
                            {
                                // Shooter is ready, go shoot.
                                robot.shooterSubsystem.shoot(moduleName, null, null);
                            }
                            else
                            {
                                robot.globalTracer.traceInfo(moduleName, "Shooter not up to speed yet.");
                            }
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "Shooter is not active, press start button to start it up.");
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
                break;

            case DpadUp:
                if (test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        subsystemName = testChoices.getSubsystemName();
                        TrcSubsystem.setSubsystemTuneTargetUp(subsystemName);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> SetTuneTargetUp: " + subsystemName);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        subsystemName = testChoices.getSubsystemName();
                        TrcSubsystem.setSubsystemTuneTargetDown(subsystemName);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> SetTuneTargetDown: " + subsystemName);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
            case DpadRight:
            case Back:
                break;

            case Start:
                if (test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        subsystemName = testChoices.getSubsystemName();
                        if (driverAltFunc)
                        {
                            TrcSubsystem.updateSubsystemParamsToDashboard(subsystemName);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Update tune param to current value.");
                        }
                        else
                        {
                            TrcSubsystem.updateSubsystemParamsFromDashboard(subsystemName);
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Start subsystem tune with the tune params.");
                        }
                    }
                    passToTeleOp = false;
                }
                else if (test == Test.TuneDriveXPid || test == Test.TuneDriveYPid || test == Test.TuneTurnPid)
                {
                    if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                robot.robotBase.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotBase.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        testChoices.getTuneXTarget()*12.0, testChoices.getTuneYTarget()*12.0,
                                        testChoices.getTuneTurnTarget()));
                            }

                            TrcPidController.PidCoefficients tunePidCoeffs = testChoices.getTunePidCoefficients();
                            if (test == Test.TuneTurnPid)
                            {
                                robot.robotBase.purePursuitDrive.setTurnPidCoefficients(tunePidCoeffs);
                            }
                            else if (robot.robotBase instanceof FrcSwerveBase)
                            {
                                robot.robotBase.purePursuitDrive.setPositionPidCoefficients(tunePidCoeffs);
                            }
                            else if (test == Test.TuneDriveXPid)
                            {
                                robot.robotBase.purePursuitDrive.setXPositionPidCoefficients(tunePidCoeffs);
                            }
                            else
                            {
                                robot.robotBase.purePursuitDrive.setYPositionPidCoefficients(tunePidCoeffs);
                            }
                            robot.robotBase.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                            robot.robotBase.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                            TrcPose2D drivePoint = tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint;
                            robot.robotBase.purePursuitDrive.start(
                                false,
                                testChoices.getMaxVelocity(),
                                testChoices.getMaxAcceleration(),
                                testChoices.getMaxDeceleration(),
                                null, drivePoint
                                );
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Pid Drive to ", drivePoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        passToTeleOp = false;
                    }
                }
                else if (test == Test.TuneShootTable && robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        if (!driverAltFunc)
                        {
                            // Update shooter velocity, pan, tilt and start it up to aim.
                            AimInfo aimInfo = new AimInfo(
                                dashboard.getNumber(DBKEY_SHOOT_VELOCITY, 0.0), null,
                                robot.shooter.panMotor != null? dashboard.getNumber(DBKEY_SHOOT_PAN_POS, 0.0): null,
                                robot.shooter.tiltMotor != null? dashboard.getNumber(DBKEY_SHOOT_TILT_POS, 0.0): null);
                            robot.shooter.aimShooter(moduleName, aimInfo);
                        }
                        else
                        {
                            // Shutdown shooter.
                            robot.shooter.cancel(moduleName);
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
            15, "TestOperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

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
     * This method calls vision code to detect target objects and display their info.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     */
    private int doVisionTest(int lineNum)
    {
        if (robot.vision != null)
        {
            lineNum = robot.vision.updateStatus(lineNum, true);
        }

        return lineNum;
    }   //doVisionTest

}   //class FrcTest
