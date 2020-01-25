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
import frclib.FrcXboxController;
import hallib.HalDashboard;
import team492.Robot.DriveSpeed;
import trclib.TrcElapsedTimer;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{

    protected final Robot robot;
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
        robot.driverController.setButtonHandler(this::driverControllerButtonEvent);
        robot.driverController.setLeftYInverted(true);

        robot.operatorStick.setButtonHandler(this::operatorStickButtonEvent);
        robot.operatorStick.setYInverted(false);

        robot.buttonPanel.setButtonHandler(this::buttonPanelButtonEvent);

        robot.switchPanel.setButtonHandler(this::switchPanelButtonEvent);

        robot.driveSpeed = DriveSpeed.MEDIUM;

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
                robot.ledIndicator.signalVision(null);
            }
            else if (pose.x > RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.ledIndicator.signalVision(LEDIndicator.Direction.RIGHT);
            }
            else if (pose.x < -RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.ledIndicator.signalVision(LEDIndicator.Direction.LEFT);
            }
            else
            {
                robot.ledIndicator.signalVision(LEDIndicator.Direction.CENTERED);
            }
        }

        HalDashboard.putString("Status/DriveSpeed", robot.driveSpeed.toString());
    }

    @Override
    public void runPeriodic(double elapsedTime)
    {
        showStatus();

        robot.updateDashboard(RunMode.TELEOP_MODE);

        //
        // DriveBase operation.
        //
        double x = robot.getXInput();
        double y = robot.getYInput();
        double rot = robot.getRotInput();
        boolean fieldOriented = robot.getFieldOriented();

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
    // Implements FrcButtonHandler.
    //

    public void driverControllerButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard.displayPrintf(8, " DriverController: button=0x%04x %s, auto=%b",
            button, pressed ? "pressed" : "released", isAutoActive);

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                if (pressed)
                {
                    robot.snapToAngle.snapToNearestAngle();
                }
                else
                {
                    robot.snapToAngle.cancel();
                }
                break;

            case FrcXboxController.BUTTON_B:
                if (pressed)
                {
                    robot.autoAlign.start(null, 0.0);
                }
                else
                {
                    robot.autoAlign.cancel();
                }
                break;

            case FrcXboxController.BUTTON_X:
                // reserved for field oriented drive in Robot.java
                break;

            case FrcXboxController.BUTTON_Y:
                break;

            case FrcXboxController.LEFT_BUMPER:
                break;

            case FrcXboxController.RIGHT_BUMPER:
                if (pressed)
                {
                    robot.autoShooter.shoot(2, 0);
                }
                else
                {
                    robot.autoShooter.cancel();
                }
                break;

            case FrcXboxController.BACK:
                break;

            case FrcXboxController.START:
                break;

            case FrcXboxController.LEFT_STICK_BUTTON:
                break;

            case FrcXboxController.RIGHT_STICK_BUTTON:
                break;
        }
    }   // driverControllerButtonEvent

    public void operatorStickButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "  OperatorStick: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            case FrcJoystick.LOGITECH_TRIGGER:
                if (pressed)
                {
                    // TODO: remove after tryouts
//                    robot.intake.intakeMultiple();
                    robot.shooter.flywheel.set(0.8);
                }
                else
                {
//                    robot.intake.stopIntake();
                    robot.shooter.flywheel.set(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                if (pressed)
                {
                    robot.intake.setIntakePower(-0.2);
                }
                else
                {
                    robot.intake.stopIntake();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    // TODO: remove after tryouts
                    robot.shooter.flywheel.set(-0.7);
                }
                else
                {
                    robot.shooter.flywheel.set(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                if (pressed)
                {
                    robot.autoShooter.shoot();
                }
                else
                {
                    robot.autoShooter.cancel();
                }
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
