package trclib;

import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class TrcLEDStripTest
{

    private TrcLEDStrip<Object> strip;
    private Object[] priority;

    @Before
    public void setup()
    {
        strip = new MockLEDStrip();
        priority = new Object[10];
        for (int i = 0; i < priority.length; i++) {
            priority[i] = new Object();
        }
        strip.setPatternPriorities(priority);
    }

    @Test
    public void getPatternPriorityTest()
    {
        for (int i = 0; i < priority.length; i++) {
            assertEquals(i, strip.getPatternPriority(priority[i]));
        }
    }

    @Test
    public void setPatternPrioritiesTest()
    {
        strip.setPatternState(priority[0], true);
        strip.setPatternState(priority[3], true);
        strip.setPatternState(priority[7], true);
        assertEquals(priority[7], strip.getPattern());

        Object[] newPriorities = new Object[7];
        System.arraycopy(priority, 0, newPriorities, 0, 4);
        for (int i = 4; i < newPriorities.length; i++){
            newPriorities[i] = new Object();
        }

        strip.setPatternPriorities(newPriorities);
        assertEquals(priority[3], strip.getPattern());
    }

    @Test
    public void startNullTest()
    {
        assertNull(strip.getPattern());
    }

    @Test
    public void resetAllPatternStatesTest()
    {
        int[] nums = new Random().ints(5, 0, priority.length).toArray();
        Arrays.stream(nums).forEach(i -> strip.setPatternState(priority[i], true));
        int max = Arrays.stream(nums).max().orElse(0);
        assertEquals(priority[max], strip.getPattern());
        strip.resetAllPatternStates();
        assertNull(strip.getPattern());
    }

    @Test
    public void setPatternStateTest()
    {
        strip.setPatternState(priority[0], true);
        assertEquals(priority[0], strip.getPattern());
        strip.setPatternState(priority[4], true);
        assertEquals(priority[4], strip.getPattern());
        strip.setPatternState(priority[2], true);
        assertEquals(priority[4], strip.getPattern());
        strip.setPatternState(priority[4], false);
        assertEquals(priority[2], strip.getPattern());
    }

    public static class MockLEDStrip extends TrcLEDStrip<Object>
    {
        private Object pattern;

        public MockLEDStrip()
        {
            super("");
        }

        @Override
        public void setPattern(Object pattern)
        {
            this.pattern = pattern;
        }

        @Override
        public Object getPattern()
        {
            return pattern;
        }
    }
}