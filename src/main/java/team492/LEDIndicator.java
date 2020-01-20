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

    private static final FrcColor visionLeftForeground = FrcColor.FULL_MAGENTA;
    private static final FrcColor visionLeftBackground = FrcColor.HALF_MAGENTA;
    private static final FrcColor visionRightForeground = FrcColor.FULL_GREEN;
    private static final FrcColor visionRightBackground = FrcColor.HALF_GREEN;
    private static final FrcColor visionCenteredForeground = FrcColor.FULL_BLUE;
    private static final FrcColor visionCenteredBackground = FrcColor.HALF_BLUE;
    private static final FrcColor noVisionForeground = FrcColor.FULL_RED;
    private static final FrcColor noVisionBackground = FrcColor.HALF_RED;

    public enum Direction
    {
        LEFT, CENTERED, RIGHT;
    }

    private Direction direction;
    private Robot robot;
    private FrcAddressableLED led;

    public LEDIndicator(Robot robot)
    {
        this.robot = robot;
        led = new FrcAddressableLED("LED", RobotInfo.PWM_CHANNEL_LED, RobotInfo.NUM_LEDS);
    }

    public void reset()
    {
        signalVision(null);
        led.setColor(noVisionBackground);
    }

    private FrcColor[] getColors()
    {
        FrcColor foreGround;
        FrcColor backGround;
        if (direction == null)
        {
            foreGround = noVisionForeground;
            backGround = noVisionBackground;
        }
        else if (direction == Direction.LEFT)
        {
            foreGround = visionLeftForeground;
            backGround = visionLeftBackground;
        }
        else if (direction == Direction.RIGHT)
        {
            foreGround = visionRightForeground;
            backGround = visionRightBackground;
        }
        else // centered
        {
            foreGround = visionCenteredForeground;
            backGround = visionCenteredBackground;
        }
        return new FrcColor[]{foreGround, backGround};
    }

    public void updateLED()
    {
        FrcColor[] colors = getColors();
        FrcColor foreGround = colors[0];
        FrcColor backGround = colors[1];

        AddressableLEDBuffer buffer = new AddressableLEDBuffer(RobotInfo.NUM_LEDS);
        int border = RobotInfo.NUM_LEDS % 5;
        int index = 0;
        for (int i = 0; i < border/2 + (border % 2 != 0 ? 1 : 0); i++)
        {
            buffer.setLED(index++, backGround);
        }
        int ledsPerBall = RobotInfo.NUM_LEDS / 5;
        for (int i = 0; i < robot.getNumBalls() * ledsPerBall; i++)
        {
            buffer.setLED(index++, foreGround);
        }
        for (int i = 0; i < (5 - robot.getNumBalls()) * ledsPerBall; i++)
        {
            buffer.setLED(index++, backGround);
        }
        for (int i = 0; i < border/2; i++)
        {
            buffer.setLED(index++, backGround);
        }

        led.setPattern(buffer);
    }

    public void signalVision(Direction direction)
    {
        this.direction = direction;
    }
}
