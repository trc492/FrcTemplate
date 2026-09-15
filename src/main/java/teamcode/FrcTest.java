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

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclib.drivebase.FrcSwerveBase;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcXboxController;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcElapsedTimer;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = FrcTest.class.getSimpleName();
    private static final boolean logEvents = false;
    private static final boolean debugPid = false;

    // Test choices.
    private static final String DBKEY_PREFIX                = "Test/";
    private static final String DBKEY_TESTS                 = DBKEY_PREFIX + "Tests";                   //Choices
    // Drive Base tuning params.
    private static final String DBKEY_DRIVE_X_TARGET        = DBKEY_PREFIX + "XTarget";                 //Number
    private static final String DBKEY_DRIVE_Y_TARGET        = DBKEY_PREFIX + "YTarget";                 //Number
    private static final String DBKEY_TURN_TARGET           = DBKEY_PREFIX + "TurnTarget";              //Number
    private static final String DBKEY_DRIVE_POWER           = DBKEY_PREFIX + "DrivePower";              //Number
    private static final String DBKEY_TURN_POWER            = DBKEY_PREFIX + "TurnPower";               //Number
    private static final String DBKEY_TIMED_DRIVE_POWER     = DBKEY_PREFIX + "TimedDrivePower";         //Number
    private static final String DBKEY_TIMED_DRIVE_TIME      = DBKEY_PREFIX + "TimedDriveTime";          //Number
    private static final String DBKEY_DRIVE_X_KP            = DBKEY_PREFIX + "DriveXKp";                //Number
    private static final String DBKEY_DRIVE_X_KI            = DBKEY_PREFIX + "DriveXKi";                //Number
    private static final String DBKEY_DRIVE_X_KD            = DBKEY_PREFIX + "DriveXKd";                //Number
    private static final String DBKEY_DRIVE_X_KF            = DBKEY_PREFIX + "DriveXKf";                //Number
    private static final String DBKEY_DRIVE_X_IZONE         = DBKEY_PREFIX + "DriveXIZone";             //Number
    private static final String DBKEY_DRIVE_Y_KP            = DBKEY_PREFIX + "DriveYKp";                //Number
    private static final String DBKEY_DRIVE_Y_KI            = DBKEY_PREFIX + "DriveYKi";                //Number
    private static final String DBKEY_DRIVE_Y_KD            = DBKEY_PREFIX + "DriveYKd";                //Number
    private static final String DBKEY_DRIVE_Y_KF            = DBKEY_PREFIX + "DriveYKf";                //Number
    private static final String DBKEY_DRIVE_Y_IZONE         = DBKEY_PREFIX + "DriveYIZone";             //Number
    private static final String DBKEY_TURN_KP               = DBKEY_PREFIX + "TurnKp";                  //Number
    private static final String DBKEY_TURN_KI               = DBKEY_PREFIX + "TurnKi";                  //Number
    private static final String DBKEY_TURN_KD               = DBKEY_PREFIX + "TurnKd";                  //Number
    private static final String DBKEY_TURN_KF               = DBKEY_PREFIX + "TurnKf";                  //Number
    private static final String DBKEY_TURN_IZONE            = DBKEY_PREFIX + "TurnIZone";               //Number
    private static final String DBKEY_VEL_KP                = DBKEY_PREFIX + "VelKp";                   //Number
    private static final String DBKEY_VEL_KI                = DBKEY_PREFIX + "VelKi";                   //Number
    private static final String DBKEY_VEL_KD                = DBKEY_PREFIX + "VelKd";                   //Number
    private static final String DBKEY_VEL_KF                = DBKEY_PREFIX + "VelKf";                   //Number
    private static final String DBKEY_VEL_IZONE             = DBKEY_PREFIX + "VelIZone";                //Number
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
    public static final String DBKEY_SUBSYSTEM_TUNE_INPUT   = DBKEY_PREFIX + "SubsystemTuneInput";      //Number
    public static final String DBKEY_SUBSYSTEM_TUNE_TARGET  = DBKEY_PREFIX + "SubsystemTuneTarget";     //Number
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
        TuneDriveBasePid,
        TuneSubsystem,
        VisionTest,
        SwerveCalibration,
        LiveWindow
    }   //enum Test

    /**
     * This class encapsulates all test choices for test mode.
     */
    public static class TestChoices
    {
        private final FrcDashboard dashboard;
        // Choice menus
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            this.dashboard = FrcDashboard.getInstance();
            //
            // Create choice menus.
            //
            testMenu = new FrcChoiceMenu<>(DBKEY_TESTS);
            //
            // Populate choice menus.
            //
            testMenu.addChoice(Test.SubsystemsTest.name(), Test.SubsystemsTest, true, false);
            testMenu.addChoice(Test.DriveSpeedTest.name(), Test.DriveSpeedTest);
            testMenu.addChoice(Test.DriveMotorsTest.name(), Test.DriveMotorsTest);
            testMenu.addChoice(Test.XTimedDrive.name(), Test.XTimedDrive);
            testMenu.addChoice(Test.YTimedDrive.name(), Test.YTimedDrive);
            testMenu.addChoice(Test.PurePursuitDrive.name(), Test.PurePursuitDrive);
            testMenu.addChoice(Test.PidDrive.name(), Test.PidDrive);
            testMenu.addChoice(Test.TuneDriveBasePid.name(), Test.TuneDriveBasePid);
            testMenu.addChoice(Test.TuneSubsystem.name(), Test.TuneSubsystem);
            testMenu.addChoice(Test.VisionTest.name(), Test.VisionTest);
            testMenu.addChoice(Test.SwerveCalibration.name(), Test.SwerveCalibration);
            testMenu.addChoice(Test.LiveWindow.name(), Test.LiveWindow, false, true);
            //
            // Publish to Dashboard (Choice menus are Choosers and don't need publishing).
            //
            dashboard.refreshKey(DBKEY_DRIVE_X_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_TURN_TARGET, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_POWER, 0.5);
            dashboard.refreshKey(DBKEY_TURN_POWER, 0.5);
            dashboard.refreshKey(DBKEY_TIMED_DRIVE_POWER, 0.5);
            dashboard.refreshKey(DBKEY_TIMED_DRIVE_TIME, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_X_KP, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_X_KI, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_X_KD, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_X_KF, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_X_IZONE, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_KP, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_KI, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_KD, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_KF, 0.0);
            dashboard.refreshKey(DBKEY_DRIVE_Y_IZONE, 0.0);
            dashboard.refreshKey(DBKEY_TURN_KP, 0.0);
            dashboard.refreshKey(DBKEY_TURN_KI, 0.0);
            dashboard.refreshKey(DBKEY_TURN_KD, 0.0);
            dashboard.refreshKey(DBKEY_TURN_KF, 0.0);
            dashboard.refreshKey(DBKEY_TURN_IZONE, 0.0);
            dashboard.refreshKey(DBKEY_VEL_KP, 0.0);
            dashboard.refreshKey(DBKEY_VEL_KI, 0.0);
            dashboard.refreshKey(DBKEY_VEL_KD, 0.0);
            dashboard.refreshKey(DBKEY_VEL_KF, 0.0);
            dashboard.refreshKey(DBKEY_VEL_IZONE, 0.0);
            dashboard.refreshKey(DBKEY_MAX_VELOCITY, 0.0);
            dashboard.refreshKey(DBKEY_MAX_ACCELERATION, 0.0);
            dashboard.refreshKey(DBKEY_MAX_DECELERATION, 0.0);
            dashboard.refreshKey(DBKEY_ROBOT_VEL, 0.0);
            dashboard.refreshKey(DBKEY_TARGET_VEL, 0.0);
            dashboard.refreshKey(DBKEY_ROBOT_POS, 0.0);
            dashboard.refreshKey(DBKEY_TARGET_POS, 0.0);
            dashboard.refreshKey(
                DBKEY_SUBSYSTEM_NAME,
                RobotParams.Preferences.testSubsystemName != null? RobotParams.Preferences.testSubsystemName: "");
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KP, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KI, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KD, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KF, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_IZONE, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_TOLERANCE, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_SOFTWARE_PID, false);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KS, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KV, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_KA, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_GRAVITY_POWER, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_TUNE_INPUT, 0.0);
            dashboard.refreshKey(DBKEY_SUBSYSTEM_TUNE_TARGET, 0.0);
            // Shoot Table tuning.
            dashboard.refreshKey(DBKEY_SHOOT_DISTANCE, 0.0);
            dashboard.refreshKey(DBKEY_SHOOT_VELOCITY, 0.0);
            dashboard.refreshKey(DBKEY_SHOOT_PAN_POS, 0.0);
            dashboard.refreshKey(DBKEY_SHOOT_TILT_POS, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getDriveXTarget()
        {
            return dashboard.getNumber(DBKEY_DRIVE_X_TARGET, 0.0);
        }   //getDriveXTarget

        public double getDriveYTarget()
        {
            return dashboard.getNumber(DBKEY_DRIVE_Y_TARGET, 0.0);
        }   //getDriveYTarget

        public double getTurnTarget()
        {
            return dashboard.getNumber(DBKEY_TURN_TARGET, 0.0);
        }   //getTurnTarget

        public double getDrivePower()
        {
            return dashboard.getNumber(DBKEY_DRIVE_POWER, 0.5);
        }   //getDrivePower

        public double getTurnPower()
        {
            return dashboard.getNumber(DBKEY_TURN_POWER, 0.5);
        }   //getTurnPower

        public double getTimedDrivePower()
        {
            return dashboard.getNumber(DBKEY_TIMED_DRIVE_POWER, 0.5);
        }   //getTImedDrivePower

        public double getTimedDriveTime()
        {
            return dashboard.getNumber(DBKEY_TIMED_DRIVE_TIME, 0.0);
        }   //getTimedDriveTime

        public TrcPidController.PidCoefficients getDriveXPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                dashboard.getNumber(DBKEY_DRIVE_X_KP, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_X_KI, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_X_KD, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_X_KF, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_X_IZONE, 0.0));
        }   //getDriveXPidCoefficients

        public void setDriveXPidCoefficients(TrcPidController.PidCoefficients pidCoeffs)
        {
            dashboard.putNumber(DBKEY_DRIVE_X_KP, pidCoeffs.kP);
            dashboard.putNumber(DBKEY_DRIVE_X_KI, pidCoeffs.kI);
            dashboard.putNumber(DBKEY_DRIVE_X_KD, pidCoeffs.kD);
            dashboard.putNumber(DBKEY_DRIVE_X_KF, pidCoeffs.kF);
            dashboard.putNumber(DBKEY_DRIVE_X_IZONE, pidCoeffs.iZone);
        }   //setDriveXPidCoefficients

        public TrcPidController.PidCoefficients getDriveYPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                dashboard.getNumber(DBKEY_DRIVE_Y_KP, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_Y_KI, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_Y_KD, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_Y_KF, 0.0),
                dashboard.getNumber(DBKEY_DRIVE_Y_IZONE, 0.0));
        }   //getDriveYPidCoefficients

        public void setDriveYPidCoefficients(TrcPidController.PidCoefficients pidCoeffs)
        {
            dashboard.putNumber(DBKEY_DRIVE_Y_KP, pidCoeffs.kP);
            dashboard.putNumber(DBKEY_DRIVE_Y_KI, pidCoeffs.kI);
            dashboard.putNumber(DBKEY_DRIVE_Y_KD, pidCoeffs.kD);
            dashboard.putNumber(DBKEY_DRIVE_Y_KF, pidCoeffs.kF);
            dashboard.putNumber(DBKEY_DRIVE_Y_IZONE, pidCoeffs.iZone);
        }   //setDriveYPidCoefficients

        public TrcPidController.PidCoefficients getTurnPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                dashboard.getNumber(DBKEY_TURN_KP, 0.0),
                dashboard.getNumber(DBKEY_TURN_KI, 0.0),
                dashboard.getNumber(DBKEY_TURN_KD, 0.0),
                dashboard.getNumber(DBKEY_TURN_KF, 0.0),
                dashboard.getNumber(DBKEY_TURN_IZONE, 0.0));
        }   //getTurnPidCoefficients

        public void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoeffs)
        {
            dashboard.putNumber(DBKEY_TURN_KP, pidCoeffs.kP);
            dashboard.putNumber(DBKEY_TURN_KI, pidCoeffs.kI);
            dashboard.putNumber(DBKEY_TURN_KD, pidCoeffs.kD);
            dashboard.putNumber(DBKEY_TURN_KF, pidCoeffs.kF);
            dashboard.putNumber(DBKEY_TURN_IZONE, pidCoeffs.iZone);
        }   //setTurnPidCoefficients

        public TrcPidController.PidCoefficients getVelPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                dashboard.getNumber(DBKEY_VEL_KP, 0.0),
                dashboard.getNumber(DBKEY_VEL_KI, 0.0),
                dashboard.getNumber(DBKEY_VEL_KD, 0.0),
                dashboard.getNumber(DBKEY_VEL_KF, 0.0),
                dashboard.getNumber(DBKEY_VEL_IZONE, 0.0));
        }   //getVelPidCoefficients

        public void setVelPidCoefficients(TrcPidController.PidCoefficients pidCoeffs)
        {
            dashboard.putNumber(DBKEY_VEL_KP, pidCoeffs.kP);
            dashboard.putNumber(DBKEY_VEL_KI, pidCoeffs.kI);
            dashboard.putNumber(DBKEY_VEL_KD, pidCoeffs.kD);
            dashboard.putNumber(DBKEY_VEL_KF, pidCoeffs.kF);
            dashboard.putNumber(DBKEY_VEL_IZONE, pidCoeffs.iZone);
        }   //setVelPidCoefficients

        public double getMaxVelocity()
        {
            return dashboard.getNumber(DBKEY_MAX_VELOCITY, 0.0);
        }   //getMaxVelocity

        public double getMaxAcceleration()
        {
            return dashboard.getNumber(DBKEY_MAX_ACCELERATION, 0.0);
        }   //getMaxAcceleration

        public double getMaxDeceleration()
        {
            return dashboard.getNumber(DBKEY_MAX_DECELERATION, 0.0);
        }   //getMaxDeceleration

        public String getSubsystemName()
        {
            return dashboard.getString(
                DBKEY_SUBSYSTEM_NAME,
                RobotParams.Preferences.testSubsystemName != null? RobotParams.Preferences.testSubsystemName: "");
        }   //getSubsystemName

        public TrcMotor.PidParams getSubsystemPidParameters()
        {
            return new TrcMotor.PidParams()
                .setPidCoefficients(
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KP, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KI, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KD, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KF, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_IZONE, 0.0))
                .setFFCoefficients(
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KS, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KV, 0.0),
                    dashboard.getNumber(DBKEY_SUBSYSTEM_KA, 0.0))
                .setPidControlParams(
                    dashboard.getNumber(DBKEY_SUBSYSTEM_TOLERANCE, 0.0),
                    dashboard.getBoolean(DBKEY_SUBSYSTEM_SOFTWARE_PID, false));
        }   //getSubsystemPidParameters

        public void setSubsystemPidParameters(TrcMotor.PidParams pidParams)
        {
            if (pidParams.pidCoeffs != null)
            {
                dashboard.putNumber(DBKEY_SUBSYSTEM_KP, pidParams.pidCoeffs.kP);
                dashboard.putNumber(DBKEY_SUBSYSTEM_KI, pidParams.pidCoeffs.kI);
                dashboard.putNumber(DBKEY_SUBSYSTEM_KD, pidParams.pidCoeffs.kD);
                dashboard.putNumber(DBKEY_SUBSYSTEM_KF, pidParams.pidCoeffs.kF);
                dashboard.putNumber(DBKEY_SUBSYSTEM_IZONE, pidParams.pidCoeffs.iZone);
            }

            if (pidParams.ffCoeffs != null)
            {
                dashboard.putNumber(DBKEY_SUBSYSTEM_KS, pidParams.ffCoeffs.kS);
                dashboard.putNumber(DBKEY_SUBSYSTEM_KV, pidParams.ffCoeffs.kV);
                dashboard.putNumber(DBKEY_SUBSYSTEM_KA, pidParams.ffCoeffs.kA);
            }

            dashboard.putNumber(DBKEY_SUBSYSTEM_TOLERANCE, pidParams.pidTolerance);
            dashboard.putBoolean(DBKEY_SUBSYSTEM_SOFTWARE_PID, pidParams.useSoftwarePid);
        }   //setSubsystemPidParameters

        @Override
        public String toString()
        {
            return String.format(
                "test=\"%s\" " +
                "xTarget=\"%.1f ft\" " +
                "yTarget=\"%.1f ft\" " +
                "turnTarget=\"%.0f deg\" " +
                "drivePower=\"%.1f\" " +
                "turnPower=\"%.1f\" " +
                "timedDrivePower=\"%.1f\" " +
                "timedDriveTime=\"%.0f sec\" " +
                "driveXPidCoeff=\"%s\" " +
                "driveYPidCoeff=\"%s\" " +
                "turnPidCoeff=\"%s\" " +
                "velPidCoeff=\"%s\" " +
                "maxVelocity=\"%.1f\" " +
                "maxAcceleration=\"%.1f\" " +
                "maxDeceleration=\"%.1f\" " +
                "subsystemName=\"%s\" " +
                "subsystemPidParams=\"%s\" ",
                getTest(), getDriveXTarget(), getDriveYTarget(), getTurnTarget(), getDrivePower(), getTurnPower(),
                getTimedDrivePower(), getTimedDriveTime(), getDriveXPidCoefficients(), getDriveYPidCoefficients(),
                getTurnPidCoefficients(), getVelPidCoefficients(), getMaxVelocity(), getMaxAcceleration(),
                getMaxDeceleration(), getSubsystemName(), getSubsystemPidParameters());
        }   //toString
    }   //class TestChoices

    //
    // Global objects.
    //
    public static final TestChoices testChoices = new TestChoices();
    private static final TrcElapsedTimer loopPerfTimer =
        RobotParams.Preferences.useLoopPerformanceMonitor? new TrcElapsedTimer("loopPerfMonitor", 2.0): null;
    private TrcRobot.RobotCommand testCommand = null;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private Double prevTime = null;
    private Double prevVelocity = null;
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
    // Extending TrcRobot.RunMode methods in FrcTeleOp.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);
        //
        // Create Command objects according to test choice.
        //
        boolean liveWindowEnabled = false;
        Test test = testChoices.getTest();

        switch (test)
        {
            case DriveSpeedTest:
                maxDriveVelocity = 0.0;
                maxDriveAcceleration = 0.0;
                maxDriveDeceleration = 0.0;
                maxTurnVelocity = 0.0;
                prevTime = null;
                prevVelocity = null;
                break;

            case DriveMotorsTest:
                if (robot.robotBase != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotBase.driveBase, robot.robotBase.driveMotors, 5.0, 0.5);
                    testCommand.start();
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
                        robot.robotBase.driveBase, 0.0, testChoices.getTimedDriveTime(), xPower, yPower, 0.0);
                    testCommand.start();
                }
                break;

            case PurePursuitDrive:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPurePursuitDrive(
                        robot.robotBase.driveBase, testChoices.getDriveXPidCoefficients(),
                        testChoices.getDriveYPidCoefficients(), testChoices.getTurnPidCoefficients(),
                        testChoices.getVelPidCoefficients());

                    ((CmdPurePursuitDrive) testCommand).startPath(
                        0.0, true,
                        testChoices.getMaxVelocity(),
                        testChoices.getMaxAcceleration(),
                        testChoices.getMaxDeceleration(),
                        testChoices.getDrivePower(),
                        testChoices.getTurnPower(),
                        new TrcPose2D(
                            testChoices.getDriveXTarget()*12.0,
                            testChoices.getDriveYTarget()*12.0,
                            testChoices.getTurnTarget()));
                    robot.robotBase.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case PidDrive:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);

                    ((CmdPidDrive) testCommand).startPath(
                        0.0, testChoices.getDrivePower(), testChoices.getTurnPower(), null,
                        new TrcPose2D(
                            testChoices.getDriveXTarget()*12.0, testChoices.getDriveYTarget()*12.0,
                            testChoices.getTurnTarget()));
                    robot.robotBase.pidDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
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
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
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
        //
        // Run the testCommand if any.
        //
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
                    Double deltaTime = prevTime == null? null: currTime - prevTime;

                    if (deltaTime != null)
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

                    if (slowPeriodicLoop)
                    {
                        robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                        robot.dashboard.displayPrintf(
                            lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                    }
                }
                break;

            case XTimedDrive:
            case YTimedDrive:
                if (slowPeriodicLoop && robot.robotBase != null)
                {
                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                    robot.dashboard.displayPrintf(
                        lineNum++, "rawEnc=fl:%.0f,fr:%.0f,bl:%.0f,br:%.0f",
                        robot.robotBase.driveMotors[MotorIndex.FrontLeft.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.BackLeft.value].getPosition(),
                        robot.robotBase.driveMotors[MotorIndex.BackRight.value].getPosition());
                }
                break;

            case TuneDriveBasePid:
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
                // Intentionally falling through.
            case PurePursuitDrive:
            case PidDrive:
                if (robot.robotBase != null && slowPeriodicLoop)
                {
                    TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                    if ((test == Test.PurePursuitDrive || test == Test.TuneDriveBasePid) &&
                        robot.robotBase.purePursuitDrive != null)
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

            case VisionTest:
                if (robot.vision != null && slowPeriodicLoop)
                {
                    lineNum = robot.vision.updateStatus(lineNum, true);
                }
                break;

            case SwerveCalibration:
                if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase && slowPeriodicLoop)
                {
                    FrcSwerveBase swerveBase = (FrcSwerveBase) robot.robotBase;
                    swerveBase.runSteeringCalibration();
                    swerveBase.displaySteerZeroCalibration(lineNum);
                }
                break;

            default:
                break;
        }
        //
        // Call super.runPeriodic only if you need TeleOp control of the robot for some tests.
        //
        if (test == Test.SubsystemsTest || test == Test.TuneSubsystem ||
            test == Test.VisionTest || test == Test.DriveSpeedTest)
        {
            super.periodic(elapsedTime, true);
        }

        if (loopPerfTimer != null)
        {
            loopPerfTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                14, "Period: %.3f(%.3f/%.3f)",
                loopPerfTimer.getAverageElapsedTime(), loopPerfTimer.getMinElapsedTime(),
                loopPerfTimer.getMaxElapsedTime());
        }
    }   //periodic

    //
    // Overriding ButtonEvent here if necessary.
    //

    /**
     * This method is called when driver gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void driverButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        Test test = testChoices.getTest();
        String tuneSubsystemName = null;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }
        robot.dashboard.displayPrintf(15, "Driver: " + button + "=" + (pressed? "pressed": "released"));
        switch (button)
        {
            case A:
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
                        tuneSubsystemName = testChoices.getSubsystemName();
                        TrcSubsystem.performTuneSubsystemAction(
                            TrcSubsystem.TuneAction.SetNextTuneTargetUp, tuneSubsystemName);
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> SetTuneTargetUp: " + tuneSubsystemName);
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (test == Test.TuneSubsystem)
                {
                    if (pressed)
                    {
                        tuneSubsystemName = testChoices.getSubsystemName();
                        TrcSubsystem.performTuneSubsystemAction(
                            TrcSubsystem.TuneAction.SetNextTuneTargetDown, tuneSubsystemName);
                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> SetTuneTargetDown: " + tuneSubsystemName);
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
                        tuneSubsystemName = testChoices.getSubsystemName();
                        if (driverAltFunc)
                        {
                            TrcSubsystem.updateSubsystemParamsToDashboard(tuneSubsystemName);
                            robot.globalTracer.traceInfo(
                                moduleName,
                                ">>>>> Update Dashboard with subsystem tune params.");
                        }
                        else
                        {
                            TrcSubsystem.updateSubsystemParamsFromDashboard(tuneSubsystemName);
                            robot.globalTracer.traceInfo(
                                moduleName,
                                ">>>>> Update subsystem tune params from Dashboard and Start subsystem tuning.");
                        }
                    }
                    passToTeleOp = false;
                }
                else if (test == Test.TuneDriveBasePid)
                {
                    if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                // At starting point.
                                robot.robotBase.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotBase.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        testChoices.getDriveXTarget()*12.0,
                                        testChoices.getDriveYTarget()*12.0,
                                        testChoices.getTurnTarget()));
                            }

                            // Update PurePursuit PID controllers from Dashboard.
                            robot.robotBase.purePursuitDrive.setXPositionPidCoefficients(
                                testChoices.getDriveXPidCoefficients());
                            robot.robotBase.purePursuitDrive.setYPositionPidCoefficients(
                                testChoices.getDriveYPidCoefficients());
                            robot.robotBase.purePursuitDrive.setTurnPidCoefficients(
                                testChoices.getTurnPidCoefficients());
                            robot.robotBase.purePursuitDrive.setVelocityPidCoefficients(
                                testChoices.getVelPidCoefficients());
                            robot.robotBase.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                            robot.robotBase.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());

                            TrcPose2D drivePoint = tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint;
                            robot.robotBase.purePursuitDrive.start(
                                false,
                                testChoices.getMaxVelocity(),
                                testChoices.getMaxAcceleration(),
                                testChoices.getMaxDeceleration(),
                                null, drivePoint);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Pid Drive to ", drivePoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        else
                        {
                            robot.robotBase.purePursuitDrive.cancel();
                        }
                        passToTeleOp = false;
                    }
                }
                break;

            default:
                break;
        }
        //
        // If the button event was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.driverButtonEvent(button, pressed);
        }
    }   //driverButtonEvent

    /**
     * This method is called when operator gamepad button event is detected.
     *
     * @param button specifies the button that generates the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void operatorButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
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
        // If the button event was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorButtonEvent(button, pressed);
        }
    }   //operatorButtonEvent

}   //class FrcTest
