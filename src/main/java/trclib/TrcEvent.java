/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

/**
 * This class implements the TrcEvent. TrcEvent is very important in our event driven architecture where things
 * only happen when an event is signaled.
 */
public class TrcEvent
{
    public enum State
    {
        CLEARED,
        SIGNALED,
        CANCELED
    }   //enum State

    private static final String moduleName = "TrcEvent";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private State state;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param state specifies the initial state of the event.
     */
    public TrcEvent(final String instanceName, State state)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.state = state;
    }   //TrcEvent

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcEvent(final String instanceName)
    {
        this(instanceName, State.CLEARED);
    }   //TrcEvent

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
     * This method returns the state of the event.
     *
     * @return state of the event.
     */
    public synchronized State get()
    {
        final String funcName = "get";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state;
    }   //get

    /**
     * This method sets the state of the event object.
     *
     * @param state specifies the event state to be set.
     */
    public synchronized void set(State state)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "state=%s", state);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.state = state;
    }   //set

    /**
     * This method signals an event if it wasn't canceled.
     */
    public synchronized void signal()
    {
        final String funcName = "signal";

        if (state == State.CLEARED)
        {
            state = State.SIGNALED;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (state=%s)", state);
        }
    }   //signal

    /**
     * This method clears an event.
     */
    public synchronized void clear()
    {
        final String funcName = "clear";

        state = State.CLEARED;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (state=%s)", state);
        }
    }   //clear

    /**
     * This method cancels an event if it is not already signaled.
     */
    public synchronized void cancel()
    {
        final String funcName = "cancel";

        if (state != State.SIGNALED)
        {
            state = State.CANCELED;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (state=%s)", state);
        }
    }   //cancel

    /**
     * This method checks if the event is signaled.
     *
     * @return true if the event is signaled, false otherwise.
     */
    public synchronized boolean isSignaled()
    {
        final String funcName = "isSignaled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state == State.SIGNALED;
    }   //isSignaled

    /**
     * This method checks if the event was canceled.
     *
     * @return true if the event was canceled, false otherwise.
     */
    public synchronized boolean isCanceled()
    {
        final String funcName = "isCanceled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
        }

        return state == State.CANCELED;
    }   //isCanceled

}   //class TrcEvent
