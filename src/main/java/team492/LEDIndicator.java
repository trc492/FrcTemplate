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

import java.util.Arrays;

public class LEDIndicator
{
    private static final FrcColor nominalColor = FrcColor.FULL_CYAN;
    private static final FrcColor fieldOrientedColor = FrcColor.FULL_WHITE;
    private static final FrcColor robotOrientedColor = FrcColor.FULL_BLUE;
    private static final FrcColor inverseOrientedColor = FrcColor.FULL_RED;

    private Robot robot;
    private FrcAddressableLED led;

    public LEDIndicator(Robot robot)
    {
        this.robot = robot;
        led = new FrcAddressableLED("LED", RobotInfo.NUM_LEDS, RobotInfo.PWM_CHANNEL_LED);
        reset();
    }

    public void reset()
    {
        led.setEnabled(true);
        led.setColor(nominalColor);
    }

    public void updateLED()
    {
        FrcColor color = nominalColor;
        switch (robot.driveOrientation)
        {
            case FIELD:
                color = fieldOrientedColor;
                break;

            case ROBOT:
                color = robotOrientedColor;
                break;

            case INVERTED:
                color = inverseOrientedColor;
                break;
        }

        setColor(color);
    }

    public void setColor(FrcColor color)
    {
        led.setColor(color);
    }
}
