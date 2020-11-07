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

package team492;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import trclib.TrcRobot.RunMode;

public class FrcTest extends FrcTeleOp
{
    //
    // Tests enum.
    //

    //
    // Test mode global objects.
    //

    //
    // Test choice menu.
    //

    //
    // Test Cmd objects.
    //

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);

        //
        // Create and initialize global objects.
        //

        //
        // Create and populate Test Mode specific menus.
        //

    } // FrcTest

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
        // Retrieve menu choice values.
        //

        boolean liveWindowEnabled = false;

        //
        // Create Cmd objects according to test choice.
        //

        LiveWindow.setEnabled(liveWindowEnabled);

        //
        // Start test state machine if necessary.
        //

    } // startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        super.stopMode(prevMode, nextMode);
    }   // stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
    } // runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        //
        // Run test Cmd.
        //

    } // runContinuous

    //
    // Overriding ButtonEvent here if necessary.
    //

    //
    // Implement tests.
    //

} // class FrcTest
