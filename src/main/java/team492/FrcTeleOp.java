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

import frclib.FrcXboxController;
import trclib.TrcRobot;
import trclib.TrcRobot.RunMode;

public class FrcTeleOp implements TrcRobot.RobotMode
{
    //
    // Global objects.
    //
    protected final Robot robot;

    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
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
        // robot.driverController.setButtonHandler(this::controllerButtonEvent);

        //
        // Initialize subsystems for TeleOp mode if necessary.
        //

    }   // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disable subsystems before exiting if necessary.
        //

    } // stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase operation.
        //
        // robot.driveBase.tankDrive(robot.driverController.getLeftYWithDeadband(true), robot.driverController.getRightYWithDeadband(true));
        robot.driveBase.arcadeDrive(robot.driverJoystick.getYWithDeadband(true), robot.driverJoystick.getTwistWithDeadband(true));

        //
        // Analog control of subsystem is done here if necessary.
        //

        //
        // Update dashboard
        //
        robot.updateDashboard(RunMode.TELEOP_MODE);
    }   // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Do subsystem auto-assist here if necessary.
        //

    } // runContinuous

    //
    // Implements FrcButtonHandler.
    //

    public void controllerButtonEvent(int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(8, "Controller: button=0x%04x %s", button, pressed ? "pressed" : "released");

        switch (button)
        {
            // case FrcXboxController.BUTTON_A:
            //     //Align using lidars
            //     if(pressed) {
            //         while(Math.abs(robot.leftLidar.getRange() - robot.rightLidar.getRange()) > 10) {
            //             if(robot.leftLidar.getRange() > robot.rightLidar.getRange()) {
            //                 robot.driveBase.tankDrive(0.5, -0.5);
            //             } else {
            //                 robot.driveBase.tankDrive(-0.5, 0.5);
            //             }
            //         }
            //     } else {
            //         robot.driveBase.tankDrive(0, 0);
            //     }
            //     break;
            // case FrcXboxController.BUTTON_B:
            //     //Turn to LimeLight
            //     if(pressed) {
            //         robot.driveBase.tankDrive(robot.limeLight.getHeading()/-100, robot.limeLight.getHeading()/100);
            //     }
            //     break;
        }
    }   // joystickButtonEvent

}   // class FrcTeleOp
