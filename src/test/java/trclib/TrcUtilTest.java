package trclib;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class TrcUtilTest
{
    @Test
    public void inRangeTest()
    {
        assertFalse(TrcUtil.inRange(3, 1, 2));
        assertTrue(TrcUtil.inRange(5, 1, 1e10));
        assertFalse(TrcUtil.inRange(25, 60, 1));
        assertTrue(TrcUtil.inRange(-1, -5, 20));
    }

    @Test
    public void roundTest()
    {
        assertEquals(1, TrcUtil.round(1.1));
        assertEquals(0, TrcUtil.round(0.0));
        assertEquals(365, TrcUtil.round(365.49));
        assertEquals(50, TrcUtil.round(49.999));
    }

    @Test
    public void maxMagnitudeTest()
    {
        assertEquals(100, TrcUtil.maxMagnitude(0.0, 3.5, 100.0), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(0.0, 1.0, 1.0), 1e-9);
        assertEquals(1, TrcUtil.maxMagnitude(1.0), 1e-9);
        assertEquals(3, TrcUtil.maxMagnitude(-2.0, -1.0, -3.0), 1e-9);
    }

    @Test
    public void averageTest()
    {
        assertEquals(0, TrcUtil.average(), 1e-9);
        assertEquals(0, TrcUtil.average(0.0), 1e-9);
        assertEquals(4, TrcUtil.average(3.0, 5.0), 1e-9);
        assertEquals(-2, TrcUtil.average(-1.0, -3.0), 1e-9);
        assertEquals(3, TrcUtil.average(3.0, 3.0, 3.0, 3.0, 3.0), 1e-9);
    }

    @Test
    public void leastSignificantSetBitTest()
    {
        assertEquals(0b1, TrcUtil.leastSignificantSetBit(0b10001));
        assertEquals(0b100, TrcUtil.leastSignificantSetBit(0b1000101010100));
        assertEquals(0b1, TrcUtil.leastSignificantSetBit(0b1010101011));
        assertEquals(0b1000, TrcUtil.leastSignificantSetBit(0b10001001000));
        assertEquals(0, TrcUtil.leastSignificantSetBit(0));
        Random r = new Random();
        for (int i = 0; i < 200; i++)
        {
            int num = r.nextInt();
            int bits = r.nextInt(30)+1;
            num &= -(1 << bits);
            num |= 1 << bits;
            assertEquals(1 << bits, TrcUtil.leastSignificantSetBit(num));
        }
    }

    @Test
    public void leastSignificantSetBitPositionTest()
    {
        assertEquals(2, TrcUtil.leastSignificantSetBitPosition(0b1000101010100));
        assertEquals(4, TrcUtil.leastSignificantSetBitPosition(0b100010101010000));
        assertEquals(-1, TrcUtil.leastSignificantSetBitPosition(0));
        Random r = new Random();
        for (int i = 0; i < 200; i++)
        {
            int num = r.nextInt();
            int bits = r.nextInt(30)+1;
            num &= -(1 << bits);
            num |= 1 << bits;
            assertEquals(bits, TrcUtil.leastSignificantSetBitPosition(num));
        }
    }

    @Test
    public void mostSignificantSetBitPositionTest()
    {
        assertEquals(5, TrcUtil.mostSignificantSetBitPosition(0b100000));
        assertEquals(6, TrcUtil.mostSignificantSetBitPosition(0b1010101));
        assertEquals(-1, TrcUtil.mostSignificantSetBitPosition(0));
        Random r = new Random();
        for (int i = 0; i < 200; i++)
        {
            int num = r.nextInt();
            int bits = r.nextInt(30)+1;
            num &= (1 << bits) - 1;
            num |= 1 << bits;
            assertEquals(bits, TrcUtil.mostSignificantSetBitPosition(num));
        }
    }
}