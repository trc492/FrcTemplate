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
import trclib.TrcAddressableLED;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern nominalPattern = new TrcAddressableLED.Pattern(FrcColor.FULL_GREEN,
        RobotInfo.NUM_LEDS);
    private static final TrcAddressableLED.Pattern fieldOrientedPattern = new TrcAddressableLED.Pattern(
        FrcColor.FULL_WHITE, RobotInfo.NUM_LEDS);
    private static final TrcAddressableLED.Pattern robotOrientedPattern = new TrcAddressableLED.Pattern(
        FrcColor.FULL_BLUE, RobotInfo.NUM_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern = new TrcAddressableLED.Pattern(
        FrcColor.FULL_RED, RobotInfo.NUM_LEDS);
    private static final TrcAddressableLED.Pattern conveyorPattern = new TrcAddressableLED.Pattern(FrcColor.FULL_YELLOW,
        RobotInfo.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] priorities = new TrcAddressableLED.Pattern[] { nominalPattern,
        fieldOrientedPattern, robotOrientedPattern, inverseOrientedPattern, conveyorPattern };

    private FrcAddressableLED led;

    public LEDIndicator()
    {
        led = new FrcAddressableLED("LED", RobotInfo.NUM_LEDS, RobotInfo.PWM_CHANNEL_LED);
        reset();
    }

    public void reset()
    {
        led.setEnabled(true);
        led.setPatternPriorities(priorities);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(nominalPattern, true);
    }

    public void setConveyorIndicator(boolean enabled)
    {
        led.setPatternState(conveyorPattern, enabled);
    }

    public void setDriveOrientation(Robot.DriveOrientation orientation)
    {
        led.setPatternState(inverseOrientedPattern, false);
        led.setPatternState(robotOrientedPattern, false);
        led.setPatternState(fieldOrientedPattern, false);
        switch (orientation)
        {
            case INVERTED:
                led.setPatternState(inverseOrientedPattern, true);
                break;

            case FIELD:
                led.setPatternState(fieldOrientedPattern, true);
                break;

            case ROBOT:
                led.setPatternState(robotOrientedPattern, true);
                break;
        }
    }
}
