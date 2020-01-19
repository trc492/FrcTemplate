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

import frclib.FrcAddressableLED;
import frclib.FrcColor;
import frclib.FrcRevBlinkin;
import trclib.TrcRevBlinkin.LEDPattern;

public class LEDIndicator
{
    private final LEDPattern normalPattern = LEDPattern.FixedBreathRed;
    private final LEDPattern visionCenteredPattern = LEDPattern.SolidGreen;
    private final LEDPattern visionLeftPattern = LEDPattern.SolidHotPink;
    private final LEDPattern visionRightPattern = LEDPattern.SolidBlue;
    private final LEDPattern[] normalPriorities = new LEDPattern[]
        { normalPattern, visionLeftPattern, visionRightPattern, visionCenteredPattern };

    private FrcAddressableLED led;

    public LEDIndicator(Robot robot)
    {
        led = new FrcAddressableLED("LED", RobotInfo.PWM_CHANNEL_LED, RobotInfo.NUM_LEDS);
        led.setColor(new FrcColor(255,0,0));
    }

//    public void enableNormalPriorities()
//    {
//        blinkin.setPatternPriorities(normalPriorities);
//    }

    public void reset()
    {
//        blinkin.resetAllPatternStates();
//        blinkin.setPatternPriorities(normalPriorities);
//        blinkin.setPatternState(normalPattern, true);
    }

    public void signalNoVisionDetected()
    {
//        blinkin.setPatternState(visionLeftPattern, false);
//        blinkin.setPatternState(visionRightPattern, false);
//        blinkin.setPatternState(visionCenteredPattern, false);
    }

    public void signalVisionLeft()
    {
        signalNoVisionDetected();
//        blinkin.setPatternState(visionLeftPattern, true);
    }

    public void signalVisionRight()
    {
        signalNoVisionDetected();
//        blinkin.setPatternState(visionRightPattern, true);
    }

    public void signalVisionCentered()
    {
        signalNoVisionDetected();
//        blinkin.setPatternState(visionCenteredPattern, true);
    }
}
