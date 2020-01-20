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

package trclib;

import java.lang.reflect.Array;
import java.util.Arrays;

public abstract class TrcLEDStrip<T>
{
    protected static final String moduleName = "TrcRevBlinkin";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private PatternState[] patternPriorities = null;

    public TrcLEDStrip(String instanceName)
    {
        this.instanceName = instanceName;

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }
    }

    /**
     * This method is provided by the platform dependent subclass that extends this class. It sets the LED pattern
     * to the physical LED strip in a platform dependent way.
     *
     * @param pattern specifies the color pattern. If null, turn off all LEDs.
     */
    public abstract void setPattern(T pattern);

    /**
     * This method is provided by the platform dependent subclass that extends this class. It gets the current set
     * LED pattern.
     *
     * @return currently set LED pattern.
     */
    public abstract T getPattern();

    /**
     * This method enables/disables the LED pattern in the priority list.
     *
     * @param pattern specifies the LED pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     */
    public void setPatternState(T pattern, boolean enabled)
    {
        final String funcName = "setPatternState";
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,state=%s,index=%d", pattern, enabled,
                index);
        }

        if (index != -1)
        {
            patternPriorities[index].enabled = enabled;
            updateLED();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPatternState

    /**
     * This method returns the LED pattern state if it is in the priority list. If the pattern is not in the list,
     * it returns false.
     *
     * @param pattern specifies the LED pattern in the priority list.
     * @return true if the LED pattern is ON, false if it is OFF.
     */
    public boolean getPatternState(T pattern)
    {
        final String funcName = "getPatternState";
        boolean state = false;
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,index=%d", pattern, index);
        }

        if (index != -1)
        {
            state = patternPriorities[index].enabled;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //getPatternState

    /**
     * This method resets all pattern states in the pattern priority list and turns off the LED strip.
     */
    public void resetAllPatternStates()
    {
        final String funcName = "resetAllPatternStates";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (patternPriorities != null)
        {
            for (PatternState state : patternPriorities)
            {
                state.enabled = false;
            }

            setPattern(null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetAllPatternStates

    /**
     * This method searches the given pattern priorities array for the given pattern. If found, its index is
     * the priority and will be returned. If the pattern is not found in the array, -1 will be return which also
     * means the lowest priority.
     *
     * @param pattern specifies the LED pattern to be searched in the pattern priorities array.
     * @return the pattern priority if found, -1 if not found.
     */
    public int getPatternPriority(T pattern)
    {
        final String funcName = "getPatternPriority";
        int priority = -1;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        if (patternPriorities != null)
        {
            for (int i = 0; i < patternPriorities.length; i++)
            {
                if (pattern == patternPriorities[i].pattern)
                {
                    priority = i;
                    break;
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", priority);
        }

        return priority;
    }   //getPatternPriority

    /**
     * This method sets the LED pattern priority list for operations that need it.
     *
     * @param priorities specifies the pattern priority list or null to disregard the previously set list.
     */
    @SuppressWarnings("unchecked")
    public void setPatternPriorities(T[] priorities)
    {
        final String funcName = "setPatternPriorities";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "priorityList=%s",
                priorities == null ? "null" : Arrays.toString(priorities));
        }

        if (priorities != null)
        {
            PatternState[] oldPriorities = patternPriorities;
            patternPriorities = (PatternState[]) Array.newInstance(PatternState.class, priorities.length);

            for (int i = 0; i < patternPriorities.length; i++)
            {
                patternPriorities[i] = new PatternState(priorities[i]);
            }

            // If we had a previous priority list, make sure patterns persist
            if (oldPriorities != null)
            {
                for (PatternState patternState : oldPriorities)
                {
                    if (patternState.enabled)
                    {
                        // This will silently fail if this pattern is not in the priority list
                        setPatternState(patternState.pattern, true);
                    }
                }
            }
            updateLED();
        }
        else
        {
            patternPriorities = null;
            setPattern(null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method is called to update the LED pattern according to the patternPriorities list. It will turn on the
     * highest priority pattern if enabled. If none of the patterns in the priority list is enabled, it will turn
     * off the LED strip.
     */
    private void updateLED()
    {
        final String funcName = "updateLED";
        T pattern = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = patternPriorities.length - 1; i >= 0; i--)
        {
            if (patternPriorities[i].enabled)
            {
                pattern = patternPriorities[i].pattern;
                break;
            }
        }
        //
        // Only set the pattern if it is not already active.
        //
        if (pattern != getPattern())
        {
            setPattern(pattern);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "! (pattern=%s)", pattern);
        }
    }   //updateLED

    /**
     * This class implements the LED pattern state.
     */
    public class PatternState
    {
        final T pattern;
        boolean enabled;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the LED pattern.
         * @param enabled specifies the initial state of the pattern.
         */
        public PatternState(T pattern, boolean enabled)
        {
            this.pattern = pattern;
            this.enabled = enabled;
        }   //PatternState

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the LED pattern.
         */
        public PatternState(T pattern)
        {
            this(pattern, false);
        }   //PatternState

    }   //class PatternState
}
