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

package frclib;

import java.util.Arrays;
import java.util.Locale;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import trclib.TrcDbgTrace;
import trclib.TrcPriorityIndicator;

/**
 * This class implements a platform dependent Addressable LED device. It uses the WPILib AddressableLED class and
 * provides methods to set the color and pattern of the LED strip.
 */
public class FrcAddressableLED extends TrcPriorityIndicator<FrcAddressableLED.Pattern>
{
    /**
     * This class contains information about an LED pattern. An LED pattern contains a pattern type, an array of colors
     * and a time interval between color changes for moving patterns.
     */
    public static class Pattern
    {
        public enum Type
        {
            SolidColor, FixedPattern, MovingColor, RotatingColor
        }   //enum PatternType

        Type type;
        FrcColor[] colors;
        double interval;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param red specifies the red value of the color in SolidColor pattern.
         * @param green specifies the green value of the color in SolidColor pattern.
         * @param blue specifies the blue value of the color in SolidColor pattern.
         */
        public Pattern(int red, int green, int blue)
        {
            this.type = Type.SolidColor;
            this.colors = new FrcColor[1];
            this.colors[0] = new FrcColor(red, green, blue);
            this.interval = 0.0;
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param color specifies the color in SolidColor pattern.
         */
        public Pattern(FrcColor color)
        {
            this.type = Type.SolidColor;
            this.colors = new FrcColor[1];
            this.colors[0] = color;
            this.interval = 0.0;
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param colors specifies an array of colors, one for each pixel in the LED strip in FixedPattern.
         */
        public Pattern(FrcColor[] colors)
        {
            this.type = Type.FixedPattern;
            this.colors = colors;
            this.interval = 0.0;
        }

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param color specifies the color in MovingColor pattern.
         * @param interval specifies the time interval in seconds between color movements.
         */
        public Pattern(FrcColor color, double interval)
        {
            this.type = Type.MovingColor;
            this.colors = new FrcColor[1];
            this.colors[0] = color;
            this.interval = interval;
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param interval specifies the time interval in seconds between color rotation.
         */
        public Pattern(double interval)
        {
            this.type = Type.RotatingColor;
            this.colors = null;
            this.interval = interval;
        }   //Pattern

        @Override
        public String toString()
        {
            return String.format(Locale.US, "type=%s,color=%s,interval=%.3f", type, Arrays.toString(colors), interval);
        }   //toString

    }   //class Pattern

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private Pattern currPattern;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel      specifies the PWM channel the device is on.
     * @param numLEDs      specifies the number of LEDs on the strip.
     */
    public FrcAddressableLED(String instanceName, int channel, int numLEDs)
    {
        super(instanceName);

        led = new AddressableLED(channel);
        led.setLength(numLEDs);
        ledBuffer = new AddressableLEDBuffer(numLEDs);
        setRGB(0, 0, 0);
    }   //FrcAddressableLED

    /**
     * This method update the LED strip to the current pattern.
     */
    private void updateLED()
    {
        switch (currPattern.type)
        {
            case SolidColor:
                for (int i = 0; i < ledBuffer.getLength(); i++)
                {
                    ledBuffer.setLED(i, currPattern.colors[0]);
                }
                led.setData(ledBuffer);
                break;

            case FixedPattern:
                int len = Math.min(currPattern.colors.length, ledBuffer.getLength());
                for (int i = 0; i < len; i++)
                {
                    ledBuffer.setLED(i, currPattern.colors[i]);
                }
                led.setData(ledBuffer);
                break;

            case MovingColor:
                // TODO: implement moving color.
                break;

            case RotatingColor:
                // TODO: implement rotating color.
                break;
        }
    }   //updateLED

    /**
     * This method sets the RGB color for the whole LED strip.
     *
     * @param red   specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue  specifies the blue value (0-255).
     */
    public void setRGB(int red, int green, int blue)
    {
        final String funcName = "setRGB";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "r=%d,g=%d,b=%d", red, green, blue);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currPattern = new Pattern(red, green, blue);
        updateLED();
    }   //setRGB

    /**
     * This method sets the HSV color for the whole LED strip.
     *
     * @param hue   specifies the hue (0-180).
     * @param sat   specifies the saturation (0-255).
     * @param value specifies the value (0-255).
     */
    public void setHSV(int hue, int sat, int value)
    {
        final String funcName = "setHSV";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "h=%d,s=%d,v=%d", hue, sat, value);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        // TODO: Implement FrcColor.hsvToRgb
        updateLED();
        // for (int i = 0; i < currPattern.getLength(); i++)
        // {
        //     currPattern.setHSV(i, hue, sat, value);
        // }

        // led.setData(currPattern);
    }   //setHSV

    /**
     * This method sets the color for the whole LED strip.
     *
     * @param color specifies the color.
     */
    public void setColor(FrcColor color)
    {
        final String funcName = "setColor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=%s", color);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currPattern = new Pattern(color);
        updateLED();
    }   //setColor

    //
    // Implements TrcPriorityIndicator abstract methods.
    //

    /**
     * This method gets the current set pattern.
     *
     * @return currently set pattern.
     */
    @Override
    public Pattern getPattern()
    {
        final String funcName = "getPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currPattern);
        }

        return currPattern;
    }   //getPattern

    /**
     * This method sets the pattern to the physical indicator device in a device dependent way.
     *
     * @param pattern specifies the indicator pattern. If null, turn off the indicator pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        if (pattern == null)
        {
            //
            // None specified, default to off pattern.
            //
            setRGB(0, 0, 0);
        }
        else
        {
            this.currPattern = pattern;
            updateLED();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPattern

}   //class FrcAddressableLED
