package team492.simulator;

import trclib.TrcDriveBase;
import trclib.TrcOdometrySensor;
import trclib.TrcUtil;

public class SimulatedHolonomicDrivebase extends TrcDriveBase
{
    private final double maxVel;
    private final double maxAccel;
    private final double maxRotVel;
    private final double maxRotAccel;
    private double xVel, yVel, rotVel;
    private double newXVel;
    private double newYVel;
    private double newRotVel;
    private Double lastTime;

    public SimulatedHolonomicDrivebase(double maxVel, double maxAccel, double maxRotVel, double maxRotAccel)
    {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxRotVel = maxRotVel;
        this.maxRotAccel = maxRotAccel;
        setSynchronizeOdometriesEnabled(false);
    }

    @Override
    protected Odometry getOdometryDelta(TrcOdometrySensor.Odometry[] prevOdometries,
        TrcOdometrySensor.Odometry[] currOdometries)
    {
        double currTime = TrcUtil.getCurrentTime();
        double dt = lastTime == null ? 0 : currTime - lastTime;
        lastTime = currTime;
        update(dt);
        Odometry o = new Odometry();
        o.position.x = xVel * dt;
        o.position.y = yVel * dt;
        o.position.angle = Math.toDegrees(rotVel * dt);
        o.velocity.x = xVel;
        o.velocity.y = yVel;
        o.velocity.angle = Math.toDegrees(rotVel);
        return o;
    }

    @Override
    public void tankDrive(String owner, double leftPower, double rightPower, boolean inverted)
    {
        if (inverted)
        {
            double temp = -rightPower;
            rightPower = -leftPower;
            leftPower = temp;
        }
        this.newXVel = 0.0;
        this.newYVel = TrcUtil.average(leftPower, rightPower) * maxVel;
        this.newRotVel = (leftPower - rightPower) * angleScale;
    }

    public void update(double dt)
    {
        xVel = rampRate(xVel, newXVel, maxAccel, dt);
        yVel = rampRate(yVel, newYVel, maxAccel, dt);
        rotVel = rampRate(rotVel, newRotVel, maxRotAccel, dt);
    }

    private double rampRate(double currVal, double newVal, double maxRate, double dt)
    {
        double rate = (newVal - currVal) / dt;
        if (Math.abs(rate) > maxRate)
        {
            newVal = currVal + Math.copySign(maxRate, rate) * dt;
        }
        return newVal;
    }

    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }

    @Override
    public void stop(String owner)
    {
        holonomicDrive(owner, 0, 0, 0, false, 0);
    }

    private double[] addNoise(double x, double y)
    {
        if (x == 0 && y == 0) return new double[2];
        double theta = Math.atan2(x, y);
        theta += (Math.random() - 0.5) * 2 * Math.toRadians(10);
        double mag = TrcUtil.magnitude(x, y);
        mag += (Math.random() - 0.5) * 2 * 0.1;
        return new double[] { mag * Math.sin(theta), mag * Math.cos(theta) };
    }

    @Override
    protected void holonomicDrive(String owner, double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        if (inverted)
        {
            x = -x;
            y = -y;
        }
        if (gyroAngle != 0)
        {
            double cosA = Math.cos(Math.toRadians(gyroAngle));
            double sinA = Math.sin(Math.toRadians(gyroAngle));
            double x1 = x * cosA - y * sinA;
            double y1 = x * sinA + y * cosA;
            x = x1;
            y = y1;
        }
        double[] noised = addNoise(x, y);
        x = noised[0];
        y = noised[1];
        newXVel = x * maxVel;
        newYVel = y * maxVel;
        newRotVel = rotation * maxRotVel;
    }
}
