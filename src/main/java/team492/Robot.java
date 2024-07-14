/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492;

import java.util.Locale;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcJoystick;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcXboxController;
import frclib.robotcore.FrcRobotBase;
import frclib.sensor.FrcPdp;
import frclib.sensor.FrcRobotBattery;
import frclib.vision.FrcPhotonVision;
import frclib.vision.FrcPhotonVisionRaw;
import team492.drivebases.RobotDrive;
import team492.drivebases.SwerveDrive;
import team492.subsystems.LEDIndicator;
import team492.vision.OpenCvVision;
import team492.vision.PhotonVision;
import team492.vision.PhotonVisionRaw;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcVisionTargetInfo;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    //
    // Global objects.
    //
    public static final String moduleName = Robot.class.getSimpleName();
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private double nextDashboardUpdateTime = TrcTimer.getCurrentTime();
    private boolean traceLogOpened = false;
    //
    // Inputs.
    //
    public FrcXboxController driverController;
    public FrcXboxController operatorController;
    public FrcJoystick leftDriveStick, rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    //
    // Miscellaneous hardware.
    //
    public LEDIndicator ledIndicator;
    //
    // Vision subsystem.
    //
    public PhotonVision photonVisionFront;
    public PhotonVision photonVisionBack;
    public PhotonVisionRaw photonVisionRaw;
    public OpenCvVision openCvVision;
    //
    // Hybrid mode objects.
    //
    public Command m_autonomousCommand;
    //
    // DriveBase subsystem.
    //
    public SwerveDrive robotDrive;
    //
    // Other subsystems.
    //

    //
    // Auto-Assists.
    //

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.Robot.NAME);
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create new hardware or subsystem, follow the steps below:
     * 1. Create a public class variable for the new hardware/subsystem.
     * 2. Instantiate and initialize the new hardware/subsystem object in this method.
     * 3. Put code in updateStatus to display status of the new hardware/subsystem if necessary.
     * 4. Put code in robotStartMode or robotStopMode to configure/reset hardware/subsystem if necessary.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     * 6. Create a getter method for the new sensor only if necessary (e.g. sensor value needs translation).
     */
    @Override
    public void robotInit()
    {
        if (RobotParams.Preferences.useCommStatusMonitor)
        {
            setCommStatusMonitorEnabled(this::commStatusCallback);
        }
        //
        // Create and initialize global objects.
        //

        //
        // Create and initialize inputs.
        //
        if (RobotParams.Preferences.useDriverXboxController)
        {
            driverController = new FrcXboxController("DriverController", RobotParams.HWConfig.XBOX_DRIVER_CONTROLLER);
            driverController.setLeftYInverted(true);
            driverController.setRightYInverted(true);
        }
        else
        {
            leftDriveStick = new FrcJoystick("DriverLeftStick", RobotParams.HWConfig.JSPORT_DRIVER_LEFTSTICK);
            leftDriveStick.setYInverted(true);
            rightDriveStick = new FrcJoystick("DriverRightStick", RobotParams.HWConfig.JSPORT_DRIVER_RIGHTSTICK);
            rightDriveStick.setYInverted(true);
        }

        if (RobotParams.Preferences.useOperatorXboxController)
        {
            operatorController = new FrcXboxController(
                "OperatorController", RobotParams.HWConfig.XBOX_OPERATOR_CONTROLLER);
            operatorController.setLeftYInverted(true);
            operatorController.setRightYInverted(true);
        }
        else
        {
            operatorStick = new FrcJoystick("operatorStick", RobotParams.HWConfig.JSPORT_OPERATORSTICK);
            operatorStick.setYInverted(false);
        }

        if (RobotParams.Preferences.useButtonPanels)
        {
            buttonPanel = new FrcJoystick("buttonPanel", RobotParams.HWConfig.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcJoystick("switchPanel", RobotParams.HWConfig.JSPORT_SWITCH_PANEL);
        }
        //
        // Create and initialize sensors.
        //
        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.HWConfig.CANID_PDP, ModuleType.kRev);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        if (RobotParams.Preferences.usePressureSensor)
        {
            pressureSensor = new AnalogInput(RobotParams.HWConfig.AIN_PRESSURE_SENSOR);
        }
        //
        // Create and initialize miscellaneous hardware.
        //
        ledIndicator = new LEDIndicator();
        //
        // Create and initialize Vision subsystem.
        //
        if (RobotParams.Preferences.useVision)
        {
            if (RobotParams.Preferences.usePhotonVision)
            {
                photonVisionFront = new PhotonVision("OV9281", RobotParams.Vision.robotToFrontCam, ledIndicator);
                photonVisionBack = new PhotonVision("OV9782", RobotParams.Vision.robotToBackCam, ledIndicator);
            }

            if (RobotParams.Preferences.usePhotonVisionRaw)
            {
                photonVisionRaw = new PhotonVisionRaw("photonvision", "OV9782", ledIndicator);
            }

            if (RobotParams.Preferences.useOpenCvVision)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(RobotParams.Vision.BACKCAM_IMAGE_WIDTH, RobotParams.Vision.BACKCAM_IMAGE_HEIGHT);
                camera.setFPS(10);
                openCvVision = new OpenCvVision(
                    "OpenCvVision", 1, RobotParams.Vision.cameraRect, RobotParams.Vision.worldRect,
                    CameraServer.getVideo(),
                    CameraServer.putVideo(
                        "UsbWebcam", RobotParams.Vision.BACKCAM_IMAGE_WIDTH, RobotParams.Vision.BACKCAM_IMAGE_HEIGHT));
            }

            if (RobotParams.Preferences.useStreamCamera)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture("DriverDisplay", 0);
                camera.setResolution(160, 120);
                camera.setFPS(10);
            }
        }
        //
        // Create and initialize RobotDrive subsystem.
        //
        robotDrive = new SwerveDrive(this, new RobotParams.SwerveDriveBase());
        //
        // Create and initialize other subsystems.
        //
        if (RobotParams.Preferences.useSubsystems)
        {
        }
        //
        // Miscellaneous.
        //
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }
        //
        // Create Auto-Assists.
        //

        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        //
        // Read FMS Match info.
        //
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
        //
        // Start trace logging.
        //
        if (runMode != RunMode.DISABLED_MODE && RobotParams.Preferences.useTraceLog)
        {
            openTraceLog(matchInfo);
            setTraceLogEnabled(true);
        }
        globalTracer.traceInfo(
            moduleName, "%s: ***** %s *****", matchInfo.eventDate, runMode);
        //
        // Start subsystems.
        //
        autoAssistCancel();

        if (robotDrive != null)
        {
            robotDrive.startMode(runMode, prevMode);
        }

        ledIndicator.reset();
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        //
        // Stop subsystems.
        //
        autoAssistCancel();
        if (robotDrive != null)
        {
            robotDrive.stopMode(runMode, nextMode);
        }
        ledIndicator.reset();
        //
        // Performance status report.
        //
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                moduleName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.HWConfig.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }
        //
        // Stop trace logging.
        //
        setTraceLogEnabled(false);
        closeTraceLog();
    }   //robotStopMode

    /**
     * This method is called periodically in the specified run mode. This is typically used to execute periodic tasks
     * that's common to all run modes.
     *
     * @param runMode specifies the current run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void robotPeriodic(RunMode runMode, boolean slowPeriodicLoop)
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // Runs the Command Based Scheduler.  This is responsible for polling buttons, adding newly-scheduled
            // commands, running already-scheduled commands, removing finished or interrupted commands, and running
            // subsystem periodic() methods.  This must be called from the robot's periodic block in order for anything
            // in the Command-based framework to work.
            CommandScheduler.getInstance().run();
        }
    }   //robotPeriodic

    /**
     * This method is called when Comm Status changes state. This is an indication of losing or regaining comm.
     *
     * @param context specifies true for comm connected, false for comm disconnected.
     */
    private void commStatusCallback(Object context)
    {
        Boolean commStatus = (Boolean) context;

        if (!commStatus)
        {
            // We lost comm, do emergency shutdown to prevent damage.
            if (robotDrive != null && robotDrive instanceof SwerveDrive)
            {
                autoAssistCancel();
                robotDrive.setXModeEnabled(null, true);
                globalTracer.traceInfo(moduleName, "***** Putting robot in X-Mode. *****");
            }
        }
    }   //commStatusCallback

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     */
    public void updateStatus()
    {
        double currTime = TrcTimer.getCurrentTime();
        RunMode runMode = getCurrentRunMode();

        if (currTime >= nextDashboardUpdateTime)
        {
            int lineNum = 9;

            nextDashboardUpdateTime = currTime + RobotParams.DASHBOARD_UPDATE_INTERVAL;

            if (RobotParams.Preferences.showPowerConsumption)
            {
                if (pdp != null)
                {
                    dashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                    dashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                    dashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                    if (runMode == RunMode.TELEOP_MODE)
                    {
                        globalTracer.traceInfo(
                            moduleName, "Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                            battery.getVoltage(), battery.getLowestVoltage());
                        globalTracer.traceInfo(moduleName, "Total=%.2fA", pdp.getTotalCurrent());
                    }
                }
            }

            if (RobotParams.Preferences.showDriveBase)
            {
                if (robotDrive != null)
                {
                    TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                    //
                    // DriveBase debug info.
                    //
                    double lfDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_LEFT_FRONT].getPosition();
                    double rfDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_FRONT].getPosition();
                    double lbDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK] != null?
                                            robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0;
                    double rbDriveEnc = robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK] != null?
                                            robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0;

                    dashboard.displayPrintf(
                        lineNum++, "DriveEnc: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);

                    if (robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robotDrive;

                        dashboard.displayPrintf(
                            lineNum++,
                            "FrontSteer(angle/motorEnc/absEnc): lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_FRONT].getSteerAngle(),
                            swerveDrive.steerMotors[RobotDrive.INDEX_LEFT_FRONT].getMotorPosition(),
                            swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_FRONT].getSteerAngle(),
                            swerveDrive.steerMotors[RobotDrive.INDEX_RIGHT_FRONT].getMotorPosition(),
                            swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_FRONT].getRawPosition());
                        dashboard.displayPrintf(
                            lineNum++,
                            "BackSteer(angle/motorEnc/absEnc): lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[RobotDrive.INDEX_LEFT_BACK].getSteerAngle(),
                            swerveDrive.steerMotors[RobotDrive.INDEX_LEFT_BACK].getMotorPosition(),
                            swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                            swerveDrive.swerveModules[RobotDrive.INDEX_RIGHT_BACK].getSteerAngle(),
                            swerveDrive.steerMotors[RobotDrive.INDEX_RIGHT_BACK].getMotorPosition(),
                            swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_BACK].getRawPosition());
                    }
                    dashboard.displayPrintf(lineNum++, "DriveBase: pose=%s", robotPose);

                    if (RobotParams.Preferences.showPidDrive)
                    {
                        TrcPidController xPidCtrl = robotDrive.pidDrive.getXPidCtrl();
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                }
            }

            if (RobotParams.Preferences.showVision)
            {
                if (photonVisionFront != null)
                {
                    FrcPhotonVision.DetectedObject object = photonVisionFront.getBestDetectedObject();
                    if (object != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "PhotonFront: pipeline=%s, obj=%s", photonVisionFront.getPipeline(), object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (photonVisionBack != null)
                {
                    FrcPhotonVision.DetectedObject object = photonVisionBack.getBestDetectedObject();
                    if (object != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "PhotonBack: pipeline=%s, obj=%s", photonVisionBack.getPipeline(), object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (photonVisionRaw != null)
                {
                    FrcPhotonVisionRaw.DetectedObject object = photonVisionRaw.getDetectedObject();
                    if (object != null)
                    {
                        dashboard.displayPrintf(lineNum++, "PhotonRaw: obj=%s", object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (openCvVision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> object =
                        openCvVision.getDetectedTargetInfo(null, null);
                    if (object != null)
                    {
                        dashboard.displayPrintf(lineNum++, "OpenCv: %s", object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }
            }

            if (RobotParams.Preferences.showSubsystems)
            {
            }
        }
    }   //updateStatus

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = TrcDbgTrace.openTraceLog(RobotParams.TEAM_FOLDER_PATH + "/tracelogs", fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method sets the drive orientation mode and update the LEDs if necessary.
     *
     * @param orientation specifies the drive orientation.
     * @param resetHeading specifies true to also reset the robot heading, only valid for FIELD mode.
     */
    public void setDriveOrientation(DriveOrientation orientation, boolean resetHeading)
    {
        if (robotDrive != null)
        {
            robotDrive.driveBase.setDriveOrientation(orientation, resetHeading);
            if (ledIndicator != null)
            {
                ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    /**
     * This method is called to cancel all pending auto-assist operations and release the ownership of all subsystems.
     */
    public void autoAssistCancel()
    {
    }   //autoAssistCancel

    /**
     * This method performs zero calibration for all subsystems.
     */
    public void zeroCalibrate()
    {
    }   //zeroCalibrate

    /**
     * This method retracts all appendages for robot high speed travelling.
     */
    public void turtle()
    {
    }   //turtle

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance.
     * @param y specifies y position in the blue alliance.
     * @param heading specifies heading in the blue alliance.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, Alliance alliance)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == Alliance.Red)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = RobotParams.Field.LENGTH - newPose.y;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, Alliance alliance)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance);
    }   //adjustPoseByAlliance

    /**
     * This method uses the detect AprilTag to relocalize the robot's position.
     *
     * @param aprilTagObj specifies the detected AprilTag object to be used for relocalization.
     * @return true if relocalization is successful, false otherwise.
     */
    public boolean relocalizeRobotByAprilTag(FrcPhotonVision.DetectedObject aprilTagObj)
    {
        boolean success = false;
        // Use AprilTag's location to re-localize the robot.
        if (aprilTagObj.robotPose != null)
        {
            robotDrive.driveBase.setFieldPosition(aprilTagObj.robotPose, false);
            globalTracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + aprilTagObj.robotPose);
            success = true;
        }

        return success;
    }   //relocalizeRobotByAprilTag

    /**
     * This method re-localizes the robot with AprilTag vision reported info.
     *
     * @param aprilTagObj specifies the detected AprilTag object.
     */
    public void relocalize(FrcPhotonVision.DetectedObject aprilTagObj)
    {
        // Use vision to relocalize robot's position.
        int aprilTagId = aprilTagObj.target.getFiducialId();
        TrcPose2D robotEstimatedPose = aprilTagObj.robotPose;

        if (robotEstimatedPose == null)
        {
            // PhotonVision pose estimator failed to return estimatedPose?! Calculate the pose ourselves.
            robotEstimatedPose = photonVisionFront.getRobotFieldPose(aprilTagObj, false);
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: aprilTagId=" + aprilTagId +
                ", robotEstimatedPoseFromAprilTag=" + robotEstimatedPose);
        }

        TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
        double xDelta = robotPose.x - robotEstimatedPose.x;
        double yDelta = robotPose.y - robotEstimatedPose.y;
        double error = TrcUtil.magnitude(xDelta, yDelta);
        if (error > RobotParams.Vision.GUIDANCE_ERROR_THRESHOLD && error < 96.00)
        {
            robotDrive.driveBase.setFieldPosition(robotEstimatedPose, false);
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: AprilTagId=" + aprilTagId + ", error=" + error +
                ", robotPose=" + robotPose + ", relocalizePose=" + robotEstimatedPose);
        }
        else
        {
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: aprilTagId=" + aprilTagId + ", error=" + error +
                " (error too large or small to relocalize).");
        }
    }   //relocalize

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return pressureSensor != null? (pressureSensor.getVoltage() - 0.5) * 50.0: 0.0;
    }   //getPressure

}   //class Robot
