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

import frclib.FrcJoystick;
import frclib.FrcRemoteVisionProcessor;
import hallib.HalDashboard;
import trclib.TrcElapsedTimer;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    private enum DriveSpeed
    {
        SLOW, MEDIUM, FAST
    }

    protected final Robot robot;
    private DriveSpeed driveSpeed = DriveSpeed.MEDIUM;
    private boolean gyroAssist = false;
    private TrcElapsedTimer elapsedTimer = null;

    public FrcTeleOp(Robot robot)
    {
        this.robot = robot;
    }   // FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Configure joysticks.
        //
        robot.leftDriveStick.setButtonHandler(this::leftDriveStickButtonEvent);
        robot.leftDriveStick.setYInverted(true);

        robot.rightDriveStick.setButtonHandler(this::rightDriveStickButtonEvent);
        robot.rightDriveStick.setYInverted(true);

        robot.operatorStick.setButtonHandler(this::operatorStickButtonEvent);
        robot.operatorStick.setYInverted(false);

        robot.buttonPanel.setButtonHandler(this::buttonPanelButtonEvent);

        robot.switchPanel.setButtonHandler(this::switchPanelButtonEvent);

        driveSpeed = DriveSpeed.MEDIUM;

        if (robot.preferences.useVision)
        {
            robot.vision.setRingLightEnabled(true);
        }

        if (robot.preferences.debugLoopTime)
        {
            elapsedTimer = new TrcElapsedTimer("TeleOpLoop", 2.0);
        }
    }   // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
    } // stopMode

    private void showStatus()
    {
        if (robot.preferences.useVision)
        {
            FrcRemoteVisionProcessor.RelativePose pose = robot.vision.getLastPose();
            HalDashboard.putBoolean("Status/TapeDetected", pose != null);
            if (pose == null)
            {
                robot.ledIndicator.signalNoVisionDetected();
            }
            else if (pose.x > RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.ledIndicator.signalVisionRight();
            }
            else if (pose.x < -RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.ledIndicator.signalVisionLeft();
            }
            else
            {
                robot.ledIndicator.signalVisionCentered();
            }
        }

        HalDashboard.putString("Status/DriveSpeed", driveSpeed.toString());
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        showStatus();

        double leftDriveX = robot.leftDriveStick.getXWithDeadband(true);
        double leftDriveY = robot.leftDriveStick.getYWithDeadband(true);
        double rightDriveY = robot.rightDriveStick.getYWithDeadband(true);
        double rightTwist = robot.rightDriveStick.getTwistWithDeadband(true);

        robot.updateDashboard(RunMode.TELEOP_MODE);
        //
        // DriveBase operation.
        //
        switch (robot.driveMode)
        {
            case HOLONOMIC_MODE:
                double x = leftDriveX;
                double y = rightDriveY;
                double rot = rightTwist;
                switch (driveSpeed)
                {
                    case SLOW:
                        x *= RobotInfo.DRIVE_SLOW_XSCALE;
                        y *= RobotInfo.DRIVE_SLOW_YSCALE;
                        rot *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                        break;

                    case MEDIUM:
                        x *= RobotInfo.DRIVE_MEDIUM_XSCALE;
                        y *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                        rot *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                        break;

                    case FAST:
                        x *= RobotInfo.DRIVE_FAST_XSCALE;
                        y *= RobotInfo.DRIVE_FAST_YSCALE;
                        rot *= RobotInfo.DRIVE_FAST_TURNSCALE;
                        break;
                }

                if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                {
                    robot.driveBase.holonomicDrive(x, y, rot, robot.driveInverted);
                }
                break;

            case TANK_MODE:
                double leftPower = leftDriveY;
                double rightPower = rightDriveY;
                switch (driveSpeed)
                {
                    case SLOW:
                        leftPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                        rightPower *= RobotInfo.DRIVE_SLOW_YSCALE;
                        break;

                    case MEDIUM:
                        leftPower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                        rightPower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                        break;

                    case FAST:
                        leftPower *= RobotInfo.DRIVE_FAST_YSCALE;
                        rightPower *= RobotInfo.DRIVE_FAST_YSCALE;
                        break;
                }

                if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                {
                    robot.driveBase.tankDrive(leftPower, rightPower, robot.driveInverted);
                }
                break;

            case ARCADE_MODE:
                double drivePower = rightDriveY;
                double turnPower = rightTwist;
                switch (driveSpeed)
                {
                    case SLOW:
                        drivePower *= RobotInfo.DRIVE_SLOW_YSCALE;
                        turnPower *= RobotInfo.DRIVE_SLOW_TURNSCALE;
                        break;

                    case MEDIUM:
                        drivePower *= RobotInfo.DRIVE_MEDIUM_YSCALE;
                        turnPower *= RobotInfo.DRIVE_MEDIUM_TURNSCALE;
                        break;

                    case FAST:
                        drivePower *= RobotInfo.DRIVE_FAST_YSCALE;
                        turnPower *= RobotInfo.DRIVE_FAST_TURNSCALE;
                        break;
                }

                if (robot.visionPidDrive == null || !robot.visionPidDrive.isActive())
                {
                    robot.driveBase.arcadeDrive(drivePower, turnPower, robot.driveInverted);
                }
                break;
        }
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                    6, "Period: %.3f(%.3f/%.3f)",
                    elapsedTimer.getAverageElapsedTime(), elapsedTimer.getMinElapsedTime(),
                    elapsedTimer.getMaxElapsedTime());
        }
    } // runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    public void leftDriveStickButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard.displayPrintf(8, " LeftDriveStick: button=0x%04x %s, auto=%b",
            button, pressed ? "pressed" : "released", isAutoActive);

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                robot.driveInverted = pressed;
                robot.setHalfBrakeModeEnabled(true);
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                driveSpeed = pressed ? DriveSpeed.FAST : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   // leftDriveStickButtonEvent

    public void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "RightDriveStick: button=0x%04x %s, auto=%b",
            button, pressed ? "pressed" : "released", robot.isAutoActive());

            switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                driveSpeed = pressed ? DriveSpeed.SLOW : DriveSpeed.MEDIUM;
                break;

            case FrcJoystick.SIDEWINDER_BUTTON2:
                robot.vision.setRingLightEnabled(!pressed);
                break;

            case FrcJoystick.SIDEWINDER_BUTTON3:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON4:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON5:
                if (pressed)
                {
                    gyroAssist = !gyroAssist;
                    if (gyroAssist)
                    {
                        robot.driveBase.enableGyroAssist(
                            RobotInfo.DRIVE_MAX_ROTATION_RATE, RobotInfo.DRIVE_GYRO_ASSIST_KP);
                    }
                    else
                    {
                        robot.driveBase.disableGyroAssist();
                    }
                }
                break;

            case FrcJoystick.SIDEWINDER_BUTTON6:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON7:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON8:
                break;

            case FrcJoystick.SIDEWINDER_BUTTON9:
                break;
        }
    }   // rightDriveStickButtonEvent

    public void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                break;

            case FrcJoystick.LOGITECH_BUTTON12:
                break;
        }
    }   // operatorStickButtonEvent

    public void buttonPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  ButtonPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_BUTTON_RED1:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN1:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE1:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW1:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE1:
                break;

            case FrcJoystick.PANEL_BUTTON_RED2:
                break;

            case FrcJoystick.PANEL_BUTTON_GREEN2:
                break;

            case FrcJoystick.PANEL_BUTTON_BLUE2:
                break;

            case FrcJoystick.PANEL_BUTTON_YELLOW2:
                break;

            case FrcJoystick.PANEL_BUTTON_WHITE2:
                break;
        }
    } // operatorStickButtonEvent

    public void switchPanelButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  SwitchPanel: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.PANEL_SWITCH_WHITE1:
                break;

            case FrcJoystick.PANEL_SWITCH_RED1:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN1:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE1:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW1:
                break;

            case FrcJoystick.PANEL_SWITCH_WHITE2:
                break;

            case FrcJoystick.PANEL_SWITCH_RED2:
                break;

            case FrcJoystick.PANEL_SWITCH_GREEN2:
                break;

            case FrcJoystick.PANEL_SWITCH_BLUE2:
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                break;
        }
    }

}   // class FrcTeleOp
