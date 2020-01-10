package team492;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import trclib.TrcUtil;

public class TrajectoryCalculator
{
    public static final double G = 9.81; // m/s^2
    public static final double METERS_PER_INCH = 0.0254;
    public static final double C = 0.5; // drag coefficient of sphere
    public static final double A = Math.PI * Math.pow(3.5 * METERS_PER_INCH, 2); // m^2, cross-sectional area of ball
    public static final double P = 1.225; // kg/m^3, density of air at sea level
    public static final double M = 0.14; // kg, mass of ball pulled from FIRST website

    /**
     * Calculate initial velocity and pitch in order to place the vertex of the trajectory at the supplied point.
     * This model does not account for air resistance.
     *
     * @param vertex 2d vector where the first entry is the robot y distance to the target, and the second is the robot
     *               z distance. Both distances should be in inches and relative to the ball exit point.
     * @return 2d vector where the first entry is initial speed and the second entry is pitch in degrees.
     */
    public static RealVector calculateWithVertexNoDrag(RealVector vertex)
    {
        vertex = vertex.mapMultiply(METERS_PER_INCH);
        double y = vertex.getEntry(0);
        double z = vertex.getEntry(0);
        // derived from basic kinematic equations
        double theta = Math.atan2(2 * z, y);
        double v = Math.sqrt(G * y * y / (2 * z * Math.pow(Math.cos(theta), 2)));
        return new ArrayRealVector(new double[] { v / METERS_PER_INCH, Math.toDegrees(theta) });
    }

    /**
     * Calculate initial velocity and pitch in order to place the vertex of the trajectory at the supplied point.
     * This model accounts for a linear model of air resistance. It's not exactly correct, but it's approximate.
     *
     * @param vertex 2d vector where the first entry is the robot y distance to the target, and the second is the robot
     *               z distance. Both distances should be in inches and relative to the ball exit point.
     * @return 2d vector where the first entry is initial speed and the second entry is pitch in degrees.
     */
    public static RealVector calculateWithVertexWithDrag(RealVector vertex)
    {
        // uses linear approximation of drag to avoid numerically solving the equation
        // then it uses a rational approximation of e^x to get a pretty accurate estimation of trajectory
        // math used from this research paper: https://www.hindawi.com/journals/ijcgt/2014/463489/
        vertex = vertex.mapMultiply(METERS_PER_INCH);
        double y = vertex.getEntry(0);
        double z = vertex.getEntry(0);
        double terminalVel = Math.sqrt(2 * M * G / (C * A * P)); // mg=1/2capv^2
        double k = G / (2 * terminalVel);
        double t = Math.sqrt(2 * z / G);
        double vy = y * (k + 1.0 / t);
        double vz = z * (k + 1.0 / t) + k * terminalVel * t;
        double v = TrcUtil.magnitude(vy, vz);
        double theta = Math.atan2(vz, vy);
        return new ArrayRealVector(new double[] { v / METERS_PER_INCH, Math.toDegrees(theta) });
    }
}
