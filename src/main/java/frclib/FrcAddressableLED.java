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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import trclib.TrcDbgTrace;
import trclib.TrcLEDStrip;

/**
 * This class implements a platform dependent Addressable LED device. It uses the WPILib AddressableLED class and
 * provides methods to set the color and pattern of the LED strip.
 */
public class FrcAddressableLED extends TrcLEDStrip<AddressableLEDBuffer>
{
    private static final String moduleName = "FrcAddressableLED";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;

    public static AddressableLEDBuffer getBufferForColor(FrcColor color, int length)
    {
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < ledBuffer.getLength(); i++)
        {
            ledBuffer.setLED(i, color);
        }
        return ledBuffer;
    }

    private TrcDbgTrace dbgTrace = null;
    private final AddressableLED led;
    private final int length;
    private AddressableLEDBuffer currPattern;

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

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        led = new AddressableLED(channel);
        led.setLength(numLEDs);
        length = numLEDs;
    }   //FrcAddressableLED

    @Override
    public void setPattern(AddressableLEDBuffer pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        if (pattern == null)
        {
            pattern = new AddressableLEDBuffer(length); // automatically all off
        }

        this.currPattern = pattern;
        led.setData(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    @Override
    public AddressableLEDBuffer getPattern()
    {
        final String funcName = "getPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currPattern);
        }

        return currPattern;
    }

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

        setColor(new FrcColor(red, green, blue));
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

        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < ledBuffer.getLength(); i++)
        {
            ledBuffer.setHSV(i, hue, sat, value);
        }

        setPattern(ledBuffer);
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

       setPattern(getBufferForColor(color, length));
    }   //setColor

}   //class FrcAddressableLED
