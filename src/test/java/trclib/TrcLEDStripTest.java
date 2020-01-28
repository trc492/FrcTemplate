package trclib;

import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class TrcLEDStripTest
{
    private TrcPriorityIndicator<Object> strip;
    private Object[] priorities;

    @Before
    public void setup()
    {
        strip = new MockLEDStrip();
        priorities = new Object[10];
        for (int i = 0; i < priorities.length; i++)
        {
            priorities[i] = new Object();
        }
        strip.setPatternPriorities(priorities);
    }

    @Test
    public void getPatternPriorityTest()
    {
        for (int i = 0; i < priorities.length; i++)
        {
            assertEquals(i, strip.getPatternPriority(priorities[i]));
        }
    }

    @Test
    public void setPatternPrioritiesTest()
    {
        strip.setPatternState(priorities[0], true);
        strip.setPatternState(priorities[3], true);
        strip.setPatternState(priorities[7], true);
        assertEquals(priorities[7], strip.getPattern());

        Object[] newPriorities = new Object[7];
        System.arraycopy(priorities, 0, newPriorities, 0, 4);
        for (int i = 4; i < newPriorities.length; i++)
        {
            newPriorities[i] = new Object();
        }

        strip.setPatternPriorities(newPriorities);
        assertEquals(priorities[3], strip.getPattern());
    }

    @Test
    public void startNullTest()
    {
        assertNull(strip.getPattern());
    }

    @Test
    public void resetAllPatternStatesTest()
    {
        int[] nums = new Random().ints(5, 0, priorities.length).toArray();
        Arrays.stream(nums).forEach(i -> strip.setPatternState(priorities[i], true));
        int max = Arrays.stream(nums).max().orElse(0);
        assertEquals(priorities[max], strip.getPattern());
        strip.resetAllPatternStates();
        assertNull(strip.getPattern());
    }

    @Test
    public void setPatternStateTest()
    {
        strip.setPatternState(priorities[0], true);
        assertEquals(priorities[0], strip.getPattern());
        strip.setPatternState(priorities[4], true);
        assertEquals(priorities[4], strip.getPattern());
        strip.setPatternState(priorities[2], true);
        assertEquals(priorities[4], strip.getPattern());
        strip.setPatternState(priorities[4], false);
        assertEquals(priorities[2], strip.getPattern());
    }

    public static class MockLEDStrip extends TrcPriorityIndicator<Object>
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