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
    private static final FrcColor visionColor = FrcColor.FULL_GREEN;
    private static final FrcColor ballColor = FrcColor.FULL_RED;
    private static final FrcColor backGround = FrcColor.FULL_WHITE;

    public enum Direction
    {
        LEFT, CENTERED, RIGHT;
    }

    private Robot robot;
    private FrcAddressableLED led;
    private FrcColor[] ledColors = new FrcColor[RobotInfo.NUM_LEDS];
    private Direction direction;

    public LEDIndicator(Robot robot)
    {
        this.robot = robot;
        led = new FrcAddressableLED("LED", RobotInfo.NUM_LEDS, RobotInfo.PWM_CHANNEL_LED);
        Arrays.fill(ledColors, backGround);
    }

    public void reset()
    {
        signalVision(null);
        led.setEnabled(true);
        led.setColor(backGround);
    }

    public void updateLED()
    {
        int index = 0;
        int ledsPerBall = RobotInfo.NUM_LEDS / 5;
        for (int i = 0; i < robot.getNumBalls() * ledsPerBall; i++)
        {
            ledColors[index++] = ballColor;
        }
        for (int i = 0; i < RobotInfo.NUM_LEDS - robot.getNumBalls() * ledsPerBall; i++)
        {
            ledColors[index++] = backGround;
        }

        if (direction != null)
        {
            int visionWidth = RobotInfo.NUM_LEDS / 8;
            int start, end;
            switch (direction)
            {
                case LEFT:
                    start = 0;
                    end = visionWidth;
                    break;

                case RIGHT:
                    start = RobotInfo.NUM_LEDS - visionWidth;
                    end = RobotInfo.NUM_LEDS;
                    break;

                default:
                case CENTERED:
                    start = RobotInfo.NUM_LEDS / 2 - visionWidth / 2;
                    end = RobotInfo.NUM_LEDS / 2 + visionWidth / 2;
                    break;
            }
            for (int i = start; i < end; i++)
            {
                ledColors[i] = visionColor;
            }
        }

        led.setPattern(new FrcAddressableLED.Pattern(ledColors));
    }

    public void setColor(FrcColor color)
    {
        led.setColor(color);
    }

    public void signalVision(Direction direction)
    {
        this.direction = direction;
        updateLED();
    }
}
