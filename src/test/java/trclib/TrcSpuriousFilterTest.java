package trclib;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TrcSpuriousFilterTest
{
    @Test
    public void toStringTest() {
        TrcSpuriousFilter Test1 = new TrcSpuriousFilter("TestFilter", 10.0);
        assertEquals("TestFilter", Test1.toString());
        TrcSpuriousFilter Test2 = new TrcSpuriousFilter("qwertyuiop", 10.0);
        assertEquals("qwertyuiop", Test2.toString());
    }

    @Test
    public void filterDataTest() {
        //Threshold is 5.0
        TrcSpuriousFilter Test1 = new TrcSpuriousFilter("TestFilter", 5.0);
        //Starting data (will return itself)
        assertEquals(0.0, Test1.filterData(0.0), 1e-10);
        //Not Spurious data, should return itself
        assertEquals(2.0, Test1.filterData(2.0), 1e-10);
        //Spurious data, should return the last data point, 2.0
        assertEquals(2.0, Test1.filterData(20.0), 1e-10);
        //Not Spurious
        assertEquals(5.5, Test1.filterData(5.5), 1e-10);
        //Not Spurious
        assertEquals(10, Test1.filterData(10), 1e-10);
        //Spurious
        assertEquals(10, Test1.filterData(Math.PI), 1e-10);
        //Not Spurious
        assertEquals(13, Test1.filterData(13), 1e-10);
        //Not Spurious
        assertEquals(15.5, Test1.filterData(15.5), 1e-10);
        //Not Spurious
        assertEquals(11, Test1.filterData(11), 1e-10);
        //Spurious
        assertEquals(11, Test1.filterData(0), 1e-10);
        //Spurious
        assertEquals(11, Test1.filterData(-356), 1e-10);
        //Not Spurious
        assertEquals(7, Test1.filterData(7), 1e-10);
        //Not Spurious
        assertEquals(3, Test1.filterData(3), 1e-10);
        //Spurious
        assertEquals(3, Test1.filterData(100), 1e-10);
    }
}