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
    private boolean lowGoal = false;
    private boolean extended = false;

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
        if (robot.preferences.useController)
        {
            robot.driverController.setButtonHandler(this::driverControllerButtonEvent);
        }
        else
        {
            robot.rightDriveStick.setButtonHandler(this::rightDriveStickButtonEvent);
        }

        robot.operatorStick.setButtonHandler(this::operatorStickButtonEvent);

        robot.buttonPanel.setButtonHandler(this::buttonPanelButtonEvent);

        robot.switchPanel.setButtonHandler(this::switchPanelButtonEvent);

        robot.driveSpeed = DriveSpeed.MEDIUM;
        robot.fieldOriented = true;
        lowGoal = false;

        if (robot.preferences.useVision)
        {
            robot.vision.setEnabled(true);
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
                robot.ledIndicator.signalVision(LEDIndicator.VisionDirection.RIGHT);
            }
            else if (pose.x < -RobotInfo.CAMERA_CENTERED_THRESHOLD)
            {
                robot.ledIndicator.signalVision(LEDIndicator.VisionDirection.LEFT);
            }
            else
            {
                robot.ledIndicator.signalVision(LEDIndicator.VisionDirection.CENTERED);
            }
        }

        robot.ledIndicator.updateLED();

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

        if (robot.shooter.isManualOverrideEnabled())
        {
            robot.shooter.setPitchPower(robot.operatorStick.getYWithDeadband(true));
        }
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(15, "Period: %.3f(%.3f/%.3f)", elapsedTimer.getAverageElapsedTime(),
                elapsedTimer.getMinElapsedTime(), elapsedTimer.getMaxElapsedTime());
        }
    } // runContinuous

    //
    // Implements FrcButtonHandler.
    //

    public void rightDriveStickButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, " RightDriveStick: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);
        switch (button)
        {
            case FrcJoystick.SIDEWINDER_TRIGGER:
                if (pressed)
                {
                    robot.fieldOriented = !robot.fieldOriented;
                }
                break;
        }
    }

    public void driverControllerButtonEvent(int button, boolean pressed)
    {
        boolean isAutoActive = robot.isAutoActive();
        robot.dashboard
            .displayPrintf(8, " DriverController: button=0x%04x %s, auto=%b", button, pressed ? "pressed" : "released",
                isAutoActive);

        switch (button)
        {
            case FrcXboxController.BUTTON_A:
                break;

            case FrcXboxController.BUTTON_B:
                String name = "AntiDefense";
                if (pressed && robot.driveBase.acquireExclusiveAccess(name))
                {
                    robot.leftFrontWheel.setSteerAngle(-45);
                    robot.rightFrontWheel.setSteerAngle(45);
                    robot.leftBackWheel.setSteerAngle(-135);
                    robot.rightBackWheel.setSteerAngle(135);
                }
                else
                {
                    robot.driveBase.releaseExclusiveAccess(name);
                    robot.leftFrontWheel.setSteerAngle(0);
                    robot.rightFrontWheel.setSteerAngle(0);
                    robot.leftBackWheel.setSteerAngle(0);
                    robot.rightBackWheel.setSteerAngle(0);
                }
                break;

            case FrcXboxController.BUTTON_X:
                if (pressed)
                {
                    robot.fieldOriented = !robot.fieldOriented;
                }
                break;

            case FrcXboxController.BUTTON_Y:
                break;

            case FrcXboxController.LEFT_BUMPER:
                if (pressed)
                {
                    robot.snapToAngle.snapToNearestAngle();
                }
                else
                {
                    robot.snapToAngle.cancel();
                }
                break;

            case FrcXboxController.RIGHT_BUMPER:
                if (pressed)
                {
                    robot.autoShooter.trackTarget();
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
                    robot.intake.intakeMultiple();
                }
                else
                {
                    robot.intake.stopIntake(false);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                // todo: toggle intake pneumatics
                if (pressed)
                {
                    extended = !extended;
                    if (extended)
                    {
                        robot.intake.extendIntake();
                    }
                    else
                    {
                        robot.intake.retractIntake();
                    }
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON3:
                if (pressed)
                {
                    if (lowGoal)
                    {
                        // if we're doing low goal, just dump the balls, we don't care about speed
                        robot.conveyor.setPower(0.8);
                    }
                    else
                    {
                        if (robot.shooter.isManualOverrideEnabled())
                        {
                            robot.shooter.setFlyWheelPower(1.0);
                        }
                        robot.conveyor.shoot();
                    }
                }
                else
                {
                    robot.conveyor.stop();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                if (pressed)
                {
                    robot.shooter.setPitch(RobotInfo.FLYWHEEL_HIGH_ANGLE);
                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_HIGH_SPEED);
                    lowGoal = false;
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                if (lowGoal = pressed)
                {
                    robot.shooter.setPitch(RobotInfo.FLYWHEEL_LOW_ANGLE);
                    robot.shooter.setFlywheelVelocity(RobotInfo.FLYWHEEL_LOW_SPEED);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
                if (pressed)
                {
                    robot.intake.setIntakePower(-0.4);
                }
                else
                {
                    robot.intake.stopIntake();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON9:
                if (pressed)
                {
                    robot.shooter.setPitch(0);
                    robot.shooter.stopFlywheel();
                }
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
                if (robot.conveyor.isManualOverrideEnabled() && pressed)
                {
                    robot.conveyor.advance();
                }
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
                robot.conveyor.setManualOverrideEnabled(pressed);
                break;

            case FrcJoystick.PANEL_SWITCH_YELLOW2:
                robot.shooter.setManualOverrideEnabled(pressed);
                break;
        }
    }

}   // class FrcTeleOp
