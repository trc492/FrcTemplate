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

import edu.wpi.first.wpilibj.GenericHID;
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

    private double deadband(double d) {
        return Math.abs(d) > 0.1 ? d : 0.0;
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        showStatus();

        robot.updateDashboard(RunMode.TELEOP_MODE);

        //
        // DriveBase operation.
        //
        double x = deadband(robot.driverController.getX(GenericHID.Hand.kLeft));
        double y = deadband(-robot.driverController.getY(GenericHID.Hand.kLeft));
        double rightTrigger = deadband(robot.driverController.getTriggerAxis(GenericHID.Hand.kRight));
        double leftTrigger = deadband(robot.driverController.getTriggerAxis(GenericHID.Hand.kLeft));
        double rot = rightTrigger > 0 ? rightTrigger : -leftTrigger;
        x = Math.copySign(Math.pow(x, 3), x);
        y = Math.copySign(Math.pow(y, 3), y);
        rot = Math.copySign(Math.pow(rot, 2), rot);
        boolean fieldOriented = robot.driverController.getXButton();

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

        robot.driveBase.holonomicDrive(x, y, rot, fieldOriented ? robot.driveBase.getHeading() : 0.0);
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
