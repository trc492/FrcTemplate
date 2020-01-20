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

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import trclib.TrcDbgTrace;

/**
 * This class implements an FRC specific color object. It extends the WPILib Color object. The WPILib color object only
 * supports RGB and not HSV. This class added HSV support to it.
 */
public class FrcColor extends Color
{
    private static final String moduleName = "FrcColor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public static final FrcColor BLACK = new FrcColor(0, 0, 0);
    public static final FrcColor FULL_RED = new FrcColor(255, 0, 0);
    public static final FrcColor FULL_GREEN = new FrcColor(0, 255, 0);
    public static final FrcColor FULL_BLUE = new FrcColor(0, 0, 255);
    public static final FrcColor FULL_YELLOW = new FrcColor(255, 255, 0);
    public static final FrcColor FULL_CYAN = new FrcColor(0, 255, 255);
    public static final FrcColor FULL_MAGENTA = new FrcColor(255, 0, 255);
    public static final FrcColor FULL_WHITE = new FrcColor(255, 255, 255);
    public static final FrcColor HALF_RED = new FrcColor(127, 0, 0);
    public static final FrcColor HALF_GREEN = new FrcColor(0, 127, 0);
    public static final FrcColor HALF_BLUE = new FrcColor(0, 0, 127);
    public static final FrcColor HALF_YELLOW = new FrcColor(127, 127, 0);
    public static final FrcColor HALF_CYAN = new FrcColor(0, 127, 127);
    public static final FrcColor HALF_MAGENTA = new FrcColor(127, 0, 127);
    public static final FrcColor HALF_WHITE = new FrcColor(127, 127, 127);

    private final double[] hsv;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param redValue specifies the red value (0-255).
     * @param greenValue specifies the green value (0-255).
     * @param blueValue specifies the blue value (0-255).
     */
    public FrcColor(int redValue, int greenValue, int blueValue)
    {
        super(new Color8Bit(redValue, greenValue, blueValue));

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + this.toString(), tracingEnabled, traceLevel, msgLevel);
        }

        hsv = rgbToHsv(redValue/255.0, greenValue/255.0, blueValue/255.0);
    }   //FrcColor

    /**
     * This method returns the string form of the RGB color values.
     *
     * @return the string form of the RGB color values.
     */
    @Override
    public String toString()
    {
        return String.format(Locale.US, "(r=%d,g=%d,b=%d", (int)(red*255), (int)(green*255), (int)(blue*255));
    }   //toString

    /**
     * This method returns the HSV color values in an array of doubles.
     *
     * @return HSV color values in an array of doubles.
     */
    public double[] getHSV()
    {
        final String funcName = "getHSV";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Arrays.toString(hsv));
        }

        return hsv;
    }   //getHSV

    /**
     * This method returns the hue component of the HSV color.
     *
     * @return the hue component of the HSV color.
     */
    public double getHue()
    {
        final String funcName = "getHue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", hsv[0]);
        }

        return hsv[0];
    }   //getHue

    /**
     * This method returns the saturation component of the HSV color.
     *
     * @return the saturation component of the HSV color.
     */
    public double getSaturation()
    {
        final String funcName = "getSaturation";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", hsv[1]);
        }

        return hsv[1];
    }   //getSaturation

    /**
     * This method returns the value component of the HSV color.
     *
     * @return the value component of the HSV color.
     */
    public double getValue()
    {
        final String funcName = "getValue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", hsv[2]);
        }

        return hsv[2];
    }   //getValue

    /**
     * This method translates the RGB color values into HSV.
     *
     * @return HSV values of the color object as an array of doubles.
     */
    public double[] rgbToHsv(double redValue, double greenValue, double blueValue)
    {
        double minValue = Math.min(Math.min(redValue, greenValue), blueValue);
        double maxValue = Math.max(Math.max(redValue, greenValue), blueValue);
        double range = maxValue - minValue;
        double hue, sat, value;

        value = maxValue;
        if (range < 0.00001)
        {
            hue = sat = 0.0;
        }
        else if (maxValue == 0.0)
        {
            sat = 0.0;
            hue = Double.NaN;
        }
        else
        {
            sat = range/maxValue;

            if (redValue == maxValue)
            {
                hue = (greenValue - blueValue)/range;
            }
            else if (greenValue == maxValue)
            {
                hue = 2.0 + (blueValue - redValue)/range;
            }
            else
            {
                hue = 4.0 + (redValue - greenValue)/range;
            }

            hue *= 60.0;
            if (hue < 0.0)
            {
                hue += 360.0;
            }
        }

        return new double[]{hue, sat, value};
     }  //rgbToHsv

 }   //class FrcColor
