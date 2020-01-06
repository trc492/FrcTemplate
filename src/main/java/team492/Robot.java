/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.SPI;
import frclib.FrcAHRSGyro;
import frclib.FrcCANSparkMax;
import frclib.FrcJoystick;
import frclib.FrcPdp;
import frclib.FrcRemoteVisionProcessor;
import frclib.FrcRobotBase;
import frclib.FrcRobotBattery;
import hallib.HalDashboard;
import trclib.TrcDigitalInput;
import trclib.TrcMecanumDriveBase;
import trclib.TrcMotor;
import trclib.TrcPidController;
import trclib.TrcPidController.PidCoefficients;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RunMode;
import trclib.TrcRobotBattery;
import trclib.TrcServo;
import trclib.TrcUtil;

import java.util.Date;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase
{
    public static class Preferences
    {
        public final boolean useTraceLog = true;
        public final boolean useNavX = true;
        public final boolean useGyroAssist = false;
        public final boolean useVision = true;
        public final boolean useStreamCamera = true;
        public final boolean doAutoUpdates = false;

        public final boolean debugPowerConsumption = false;
        public final boolean debugDriveBase = false;
        public final boolean debugPidDrive = false;
        public final boolean debugSubsystems = false;
        public final boolean debugVision = false;
        public final boolean debugLoopTime = true;
    }   //class Preferences

    public enum DriveMode
    {
        HOLONOMIC_MODE, TANK_MODE, ARCADE_MODE
    }   // enum DriveMode
    //
    // Global constants.
    //
    public static final String programName = "InfiniteRecharge";
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;
    //
    // Global objects.
    //
    public final Preferences preferences = new Preferences();
    public final DriverStation ds = DriverStation.getInstance();
    public final HalDashboard dashboard = HalDashboard.getInstance();
    //
    // Inputs.
    //
    public FrcJoystick leftDriveStick;
    public FrcJoystick rightDriveStick;
    public FrcJoystick operatorStick;
    public FrcJoystick buttonPanel;
    public FrcJoystick switchPanel;
    //
    // Sensors.
    //
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public FrcAHRSGyro gyro;
    public AnalogInput pressureSensor;
    //
    // DriveBase subsystem.
    //
    public FrcCANSparkMax leftFrontWheel;
    public FrcCANSparkMax leftBackWheel;
    public FrcCANSparkMax rightFrontWheel;
    public FrcCANSparkMax rightBackWheel;

    public TrcMecanumDriveBase driveBase;

    public DriveMode driveMode;
    public boolean driveInverted;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;
    //
    // Vision subsystem.
    //
    public VisionTargeting vision = null;

    public TrcPidController visionXPidCtrl = null;
    public TrcPidController visionYPidCtrl = null;
    public TrcPidController visionTurnPidCtrl = null;
    public TrcPidDrive visionPidDrive = null;
    //
    // Miscellaneous subsystem.
    //
    public LEDIndicator ledIndicator;

    private double nextUpdateTime = TrcUtil.getCurrentTime();
    //
    // FMS provided the following info:
    //  - event name
    //  - match type
    //  - match number
    //  - alliance
    //  - location
    //  - replay number???
    //
    public String eventName = "Unknown";
    public MatchType matchType = MatchType.None;
    public int matchNumber = 0;
    public Alliance alliance = Alliance.Red;
    public int location = 1;
    public String gameSpecificMessage = null;
    public boolean traceLogOpened = false;
    //
    // Other robot subystems.
    //

    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double turnDegrees;
    public double drivePowerLimit;
    public TrcPidController.PidCoefficients tunePidCoeff;

    private FrcAuto autoMode;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //
        // Inputs.
        //
        leftDriveStick = new FrcJoystick("leftDriveStick", RobotInfo.JSPORT_LEFT_DRIVESTICK);
        rightDriveStick = new FrcJoystick("rightDriveStick", RobotInfo.JSPORT_RIGHT_DRIVESTICK);
        operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK);
        buttonPanel = new FrcJoystick("buttonPanel", RobotInfo.JSPORT_BUTTON_PANEL);
        switchPanel = new FrcJoystick("switchPanel", RobotInfo.JSPORT_SWITCH_PANEL);
        //
        // Sensors.
        //
        pdp = new FrcPdp(RobotInfo.CANID_PDP);
        battery = new FrcRobotBattery(pdp);
        gyro = preferences.useNavX? new FrcAHRSGyro("NavX", SPI.Port.kMXP): null;
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);
        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FrcCANSparkMax("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL, true);
        leftBackWheel = new FrcCANSparkMax("LeftBackWheel", RobotInfo.CANID_LEFTBACKWHEEL, true);
        rightFrontWheel = new FrcCANSparkMax("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL, true);
        rightBackWheel = new FrcCANSparkMax("RightBackWheel", RobotInfo.CANID_RIGHTBACKWHEEL, true);

        leftFrontWheel.setInverted(false);
        leftBackWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightBackWheel.setInverted(true);

        leftFrontWheel.setPositionSensorInverted(false);
        leftBackWheel.setPositionSensorInverted(false);
        rightFrontWheel.setPositionSensorInverted(false);
        rightBackWheel.setPositionSensorInverted(false);

        leftFrontWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        rightFrontWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        leftBackWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        rightBackWheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);

        pdp.registerEnergyUsed(
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_FRONT_WHEEL, "LeftFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_LEFT_BACK_WHEEL, "LeftBackWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_FRONT_WHEEL, "RightFrontWheel"),
            new FrcPdp.Channel(RobotInfo.PDP_CHANNEL_RIGHT_BACK_WHEEL, "RightBackWheel"));

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, gyro);
        setHalfBrakeModeEnabled(true); // Karkeys prefers front coast, back brake
        driveBase.setOdometryScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        driveInverted = false;
        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController("encoderXPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_X_KP_SMALL, RobotInfo.ENCODER_X_KI_SMALL,
                RobotInfo.ENCODER_X_KD_SMALL, RobotInfo.ENCODER_X_KF_SMALL), RobotInfo.ENCODER_X_TOLERANCE_SMALL,
            driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController("encoderYPidCtrl",
            new PidCoefficients(RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD,
                RobotInfo.ENCODER_Y_KF), RobotInfo.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroTurnPidCtrl = new TrcPidController("gyroTurnPidCtrl",
            new PidCoefficients(RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD,
                RobotInfo.GYRO_TURN_KF), RobotInfo.GYRO_TURN_TOLERANCE, driveBase::getHeading);
        encoderXPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_XPID_POWER);
        encoderYPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_YPID_POWER);
        gyroTurnPidCtrl.setOutputLimit(RobotInfo.DRIVE_MAX_TURNPID_POWER);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        encoderXPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_XPID_RAMP_RATE);
        encoderYPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_YPID_RAMP_RATE);
        gyroTurnPidCtrl.setRampRate(RobotInfo.DRIVE_MAX_TURNPID_RAMP_RATE);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallTimeout(RobotInfo.DRIVE_STALL_TIMEOUT);
        pidDrive.setMsgTracer(globalTracer);
        //
        // Vision subsystem.
        //
        if (preferences.useVision)
        {
            vision = new VisionTargeting(preferences);
            visionXPidCtrl = new TrcPidController("visionXPidCtrl",
                new PidCoefficients(RobotInfo.VISION_X_KP, RobotInfo.VISION_X_KI, RobotInfo.VISION_X_KD),
                RobotInfo.VISION_X_TOLERANCE, this::getVisionX);
            visionYPidCtrl = new TrcPidController("visionYPidCtrl",
                new PidCoefficients(RobotInfo.VISION_Y_KP, RobotInfo.VISION_Y_KI, RobotInfo.VISION_Y_KD),
                RobotInfo.VISION_Y_TOLERANCE, this::getVisionY);
            visionTurnPidCtrl = new TrcPidController("visionTurnPidCtrl",
                new PidCoefficients(RobotInfo.VISION_TURN_KP, RobotInfo.VISION_TURN_KI, RobotInfo.VISION_TURN_KD),
                RobotInfo.VISION_TURN_TOLERANCE, this::getVisionYaw);
            visionXPidCtrl.setInverted(true);
            visionXPidCtrl.setAbsoluteSetPoint(true);
            visionYPidCtrl.setInverted(true);
            visionYPidCtrl.setAbsoluteSetPoint(true);
            visionTurnPidCtrl.setInverted(true);
            visionTurnPidCtrl.setAbsoluteSetPoint(true);
            visionPidDrive = new TrcPidDrive(
                "visionPidDrive", driveBase, visionXPidCtrl, visionYPidCtrl, visionTurnPidCtrl);
            visionPidDrive.setMsgTracer(globalTracer);
        }
        //
        // Miscellaneous subsystems.
        //
        ledIndicator = new LEDIndicator(this);

        if (preferences.useStreamCamera)
        {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("DriverDisplay", 0);
            camera.setResolution(160, 120);
        }
        //
        // Other robot subsystems.
        //

        //
        // AutoAssist commands.
        //

        //
        // Create Robot Modes.
        //
        autoMode = new FrcAuto(this);
        setupRobotModes(new FrcTeleOp(this), autoMode, new FrcTest(this), new FrcDisabled(this));

        pdp.registerEnergyUsedForAllUnregisteredChannels();
    }   //robotInit

    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        final String funcName = "robotStartMode";

        if (runMode != RunMode.DISABLED_MODE)
        {
            openTraceLog(
                runMode == RunMode.AUTO_MODE? "FrcAuto": runMode == RunMode.TELEOP_MODE? "FrcTeleOp": "FrcTest");
            setTraceLogEnabled(true);

            Date now = new Date();
            globalTracer.traceInfo(
                funcName, "[%.3f] %s: ***** %s *****", TrcUtil.getModeElapsedTime(), now.toString(), runMode);

            driveInverted = false;

            pdp.setTaskEnabled(true);
            battery.setEnabled(true);
            setVisionEnabled(true);
            driveBase.resetOdometry(true, false);
            driveBase.setOdometryEnabled(true);

            dashboard.clearDisplay();

            ledIndicator.reset();

            if (runMode == RunMode.AUTO_MODE || runMode == RunMode.TEST_MODE)
            {
                driveTime = HalDashboard.getNumber("Test/DriveTime", 5.0);
                drivePower = HalDashboard.getNumber("Test/DrivePower", 0.2);
                driveDistance = HalDashboard.getNumber("Test/DriveDistance", 6.0);
                turnDegrees = HalDashboard.getNumber("Test/TurnDegrees", 90.0);
                drivePowerLimit = HalDashboard.getNumber("Test/DrivePowerLimit", 0.5);
                if (runMode == RunMode.TEST_MODE)
                {
                    tunePidCoeff = new TrcPidController.PidCoefficients(
                        HalDashboard.getNumber("Test/TuneKp", RobotInfo.GYRO_TURN_KP),
                        HalDashboard.getNumber("Test/TuneKi", RobotInfo.GYRO_TURN_KI),
                        HalDashboard.getNumber("Test/TuneKd", RobotInfo.GYRO_TURN_KD),
                        HalDashboard.getNumber("Test/TuneKf", 0.0));
                }
            }

            gyro.setElapsedTimerEnabled(true);
            TrcDigitalInput.setElapsedTimerEnabled(true);
            TrcMotor.setElapsedTimerEnabled(true);
            TrcServo.setElapsedTimerEnabled(true);
        }
    }   //robotStartMode

    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        final String funcName = "robotStopMode";

        if (runMode != RunMode.DISABLED_MODE)
        {
            gyro.printElapsedTime(globalTracer);
            gyro.setElapsedTimerEnabled(false);
            TrcDigitalInput.printElapsedTime(globalTracer);
            TrcDigitalInput.setElapsedTimerEnabled(false);
            TrcMotor.printElapsedTime(globalTracer);
            TrcMotor.setElapsedTimerEnabled(false);
            TrcServo.printElapsedTime(globalTracer);
            TrcServo.setElapsedTimerEnabled(false);

            globalTracer.traceInfo(funcName, "mode=%s,heading=%.1f", runMode.name(), driveBase.getHeading());
            driveBase.setOdometryEnabled(false);
            setVisionEnabled(false);
            cancelAllAuto();
            battery.setEnabled(false);
            pdp.setTaskEnabled(false);

            for (int i = 0; i < FrcPdp.kPDPChannels; i++)
            {
                String channelName = pdp.getChannelName(i);
                if (channelName != null)
                {
                    globalTracer.traceInfo(
                        funcName, "[PDP-%02d] %s: EnergyUsed=%.3f Wh", i, channelName, pdp.getEnergyUsed(i));
                }
            }

            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(funcName, "TotalEnergy=%.3fWh (%.2f%%)", totalEnergy,
                totalEnergy * 100.0 / RobotInfo.BATTERY_CAPACITY_WATT_HOUR);
            setTraceLogEnabled(false);
            closeTraceLog();

            ledIndicator.reset();
        }
    }   //robotStopMode

    public void getFMSInfo()
    {
        eventName = ds.getEventName();
        if (eventName.length() == 0)
        {
            eventName = "Unknown";
        }
        matchType = ds.getMatchType();
        matchNumber = ds.getMatchNumber();
    }

    public void getGameInfo()
    {
        alliance = ds.getAlliance();
        location = ds.getLocation();
        gameSpecificMessage = ds.getGameSpecificMessage();
    }

    public void openTraceLog(String defaultName)
    {
        if (preferences.useTraceLog && !traceLogOpened)
        {
            String fileName;

            if (ds.isFMSAttached())
            {
                getFMSInfo();
                fileName = String.format("%s_%s%03d", eventName, matchType, matchNumber);
            }
            else
            {
                fileName = defaultName;
            }

            traceLogOpened = globalTracer.openTraceLog("/home/lvuser/tracelog", fileName);
        }
    }

    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            globalTracer.closeTraceLog();
            traceLogOpened = false;
        }
    }

    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            globalTracer.setTraceLogEnabled(enabled);
        }
    }

    public void enableSmallGains()
    {
        encoderXPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.ENCODER_X_KP_SMALL, RobotInfo.ENCODER_X_KI_SMALL,
                RobotInfo.ENCODER_X_KD_SMALL, RobotInfo.ENCODER_X_KF_SMALL));
        encoderXPidCtrl.setTargetTolerance(RobotInfo.ENCODER_X_TOLERANCE_SMALL);
        gyroTurnPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.GYRO_TURN_KP_SMALL, RobotInfo.GYRO_TURN_KI_SMALL,
                RobotInfo.GYRO_TURN_KD_SMALL));
        gyroTurnPidCtrl.setTargetTolerance(RobotInfo.GYRO_TURN_TOLERANCE_SMALL);
    }

    public void enableBigGains()
    {
        encoderXPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD,
                RobotInfo.ENCODER_X_KF));
        encoderXPidCtrl.setTargetTolerance(RobotInfo.ENCODER_X_TOLERANCE);
        gyroTurnPidCtrl.setPidCoefficients(
            new PidCoefficients(RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD));
        gyroTurnPidCtrl.setTargetTolerance(RobotInfo.GYRO_TURN_TOLERANCE);
    }

    /**
     * Stops the lower level subsystems. This does NOT stop any auto/auto assist commands.
     */
    public void stopSubsystems()
    {
        pidDrive.cancel();
        driveBase.stop();
    }

    public void setVisionEnabled(boolean enabled)
    {
    }   //setVisionEnabled

    public void updateDashboard(RunMode runMode)
    {
        final String funcName = "updateDashboard";
        double currTime = TrcUtil.getModeElapsedTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            if (preferences.debugPowerConsumption)
            {
                HalDashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                HalDashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                HalDashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                if (runMode == RunMode.TELEOP_MODE)
                {
                    globalTracer.traceInfo(funcName, "[%.3f] Battery: currVoltage=%.2f, lowestVoltage=%.2f", currTime,
                        battery.getVoltage(), battery.getLowestVoltage());
                    globalTracer.traceInfo(funcName, "[%.3f] Total=%.2fA", currTime, pdp.getTotalCurrent());
                }
            }

            if (preferences.debugDriveBase)
            {
                double xPos = driveBase.getXPosition();
                double yPos = driveBase.getYPosition();
                double heading = driveBase.getHeading();

                HalDashboard.putNumber("DriveBase/xPos", xPos);
                HalDashboard.putNumber("DriveBase/yPos", yPos);
                HalDashboard.putData("DriveBase/heading", gyro.getGyroSendable());

                //
                // DriveBase debug info.
                //
                double lfEnc = leftFrontWheel.getPosition();
                double rfEnc = rightFrontWheel.getPosition();
                double lbEnc = leftBackWheel.getPosition();
                double rbEnc = rightBackWheel.getPosition();

                dashboard
                    .displayPrintf(8, "DriveBase: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f", lfEnc, rfEnc, lbEnc,
                        rbEnc, (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                dashboard.displayPrintf(9, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f", xPos, yPos, heading);

                if (preferences.debugPidDrive)
                {
                    encoderXPidCtrl.displayPidInfo(10);
                    encoderYPidCtrl.displayPidInfo(12);
                    gyroTurnPidCtrl.displayPidInfo(14);
                }
            }

            if (preferences.debugSubsystems)
            {
                if (preferences.debugVision && vision != null)
                {
                    FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
                    if (pose != null)
                    {
                        dashboard.displayPrintf(13, "VisionTarget: x=%.1f,y=%.1f,objectYaw=%.1f", pose.x, pose.y,
                            pose.objectYaw);
                    }
                    else
                    {
                        dashboard.displayPrintf(13, "VisionTarget: No target found!");
                    }
                }
            }
        }
    }   //updateDashboard

    /**
     * Checks if any auto processes are running, be it auto mode or auto assist, etc.
     *
     * @return True if any auto processes are active, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoMode.isAutoActive();
    }

    public void cancelAllAuto()
    {
        if (autoMode.isAutoActive())
        {
            autoMode.cancel();
        }
    }

    public void traceStateInfo(double elapsedTime, String stateName, double xTarget, double yTarget, double turnTarget)
    {
        final String funcName = "traceStateInfo";
        StringBuilder msg = new StringBuilder();

        msg.append(String.format(
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                elapsedTime, stateName,
                driveBase.getXPosition(), xTarget,
                driveBase.getYPosition(), yTarget,
                driveBase.getHeading(), turnTarget));

        if (battery != null)
        {
            msg.append(String.format(",volt=%5.2fV(%5.2fV)", battery.getVoltage(), battery.getLowestVoltage()));
        }

        globalTracer.traceInfo(funcName, "%s", msg);
    }   //traceStateInfo

    public void setHalfBrakeModeEnabled(boolean enabled)
    {
        if (enabled)
        {
            leftFrontWheel.setBrakeModeEnabled(driveInverted);
            rightFrontWheel.setBrakeModeEnabled(driveInverted);
            leftBackWheel.setBrakeModeEnabled(!driveInverted);
            rightBackWheel.setBrakeModeEnabled(!driveInverted);
        }
        else
        {
            leftFrontWheel.setBrakeModeEnabled(true);
            rightFrontWheel.setBrakeModeEnabled(true);
            leftBackWheel.setBrakeModeEnabled(true);
            rightBackWheel.setBrakeModeEnabled(true);
        }
    }

    //
    // Getters for sensor data.
    //

    public double getPressure()
    {
        return (pressureSensor.getVoltage() - 0.5) * 50.0;
    }   //getPressure

    public double getVisionX()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            return pose.x;
        }
        return 0.0;
    }

    public double getVisionY()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            double outputLimit = Math.abs(rightDriveStick.getYWithDeadband(true));
            visionYPidCtrl.setOutputRange(-outputLimit, outputLimit);
            return pose.y;
        }
        return 0.0;
        // Alternate implementation:
        // This implementation will allow the joystick Y to fully control the Y direction of the PID drive.
        // So vision target has nothing to do with the Y direction.
        // return -rightDriveStick.getYWithDeadband(true)/RobotInfo.VISION_Y_KP;
    }

    public double getVisionYaw()
    {
        FrcRemoteVisionProcessor.RelativePose pose = vision.getLastPose();
        if (pose != null)
        {
            return pose.objectYaw;
        }
        return 0.0;
    }

}   //class Robot
