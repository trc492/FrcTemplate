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

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frclib.FrcAddressableLED;
import frclib.FrcColor;

public class LEDIndicator
{

    private final AddressableLEDBuffer defaultPattern = getBuffer(FrcColor.BLACK);
    private final AddressableLEDBuffer visionLeftPattern = getBuffer(FrcColor.FULL_RED);
    private final AddressableLEDBuffer visionCenteredPattern = getBuffer(FrcColor.FULL_GREEN);
    private final AddressableLEDBuffer visionRightPattern = getBuffer(FrcColor.FULL_BLUE);
    private final AddressableLEDBuffer[] priorities = { defaultPattern, visionLeftPattern, visionRightPattern,
        visionCenteredPattern };

    private static AddressableLEDBuffer getBuffer(FrcColor color)
    {
        return FrcAddressableLED.getBufferForColor(color, RobotInfo.NUM_LEDS);
    }

    private FrcAddressableLED led;

    public LEDIndicator()
    {
        led = new FrcAddressableLED("LED", RobotInfo.PWM_CHANNEL_LED, RobotInfo.NUM_LEDS);
        led.setPatternPriorities(priorities);
        reset();
    }

    public void reset()
    {
        led.resetAllPatternStates();
        led.setPatternPriorities(priorities);
        led.setPatternState(defaultPattern, true);
    }

    public void signalNoVisionDetected()
    {
        reset();
        led.setPatternState(visionLeftPattern, false);
        led.setPatternState(visionRightPattern, false);
        led.setPatternState(visionCenteredPattern, false);
    }

    public void signalVisionLeft()
    {
        led.setPatternState(visionLeftPattern, true);
    }

    public void signalVisionRight()
    {
        led.setPatternState(visionRightPattern, true);
    }

    public void signalVisionCentered()
    {
        led.setPatternState(visionCenteredPattern, true);
    }
}
