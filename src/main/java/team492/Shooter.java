package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frclib.FrcCANFalcon;
import frclib.FrcCANTalon;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Shooter
{
    private static final double FLYWHEEL_kP = 0.05;
    private static final double FLYWHEEL_kI = 1e-4;
    private static final int FLYWHEEL_IZONE = 2000;
    private static final double FLYWHEEL_kD = 5;
    private static final double FLYWHEEL_kF = 0.0479; //1.0 / RobotInfo.FLYWHEEL_TOP_SPEED;

    private static final boolean USE_MM = true;
    private static final double PITCH_kP = 3;
    private static final double PITCH_kI = 0.0004;
    private static final double PITCH_kD = 250;
    private static final double PITCH_kF = 1.202;
    private static final double PITCH_MAX_POWER = 0.8;
    private static final int PITCH_IZONE = 100;
    private static final int PITCH_MAX_VEL = 250;
    private static final int PITCH_MAX_ACCEL = 550;
    private static final double PITCH_DEGREES_PER_COUNT = 360.0 / 4096.0 / 1.5; // 3:2 gear ratio, 4096 cpr
    private static final double PITCH_TOLERANCE = 2.5;
    private static final int PITCH_ALLOWABLE_ERROR = (int) (1 / PITCH_DEGREES_PER_COUNT);

    // Measured
    private static final int PITCH_OFFSET_TICKS = 232;
    private static final double PITCH_OFFSET_DEG = -3.2;
    private static final int PITCH_UPPER_LIMIT_TICKS = (int) (48 / PITCH_DEGREES_PER_COUNT); // gosh darn G17
    private static final int PITCH_LOWER_LIMIT_TICKS = (int) (0 / PITCH_DEGREES_PER_COUNT);

    public final FrcCANFalcon flywheel;
    public final FrcCANTalon pitchMotor;
    private TrcTaskMgr.TaskObject pitchControlTaskObj;
    private TrcEvent pitchEvent;
    private Integer pitchTicksTarget;
    private boolean manualOverride;
    private Robot robot;

    public Shooter(Robot robot)
    {
        this.robot = robot;
        flywheel = new FrcCANFalcon("Shooter.flywheel", RobotInfo.CANID_FLYWHEEL);
        pitchMotor = new FrcCANTalon("Shooter.pitchMotor", RobotInfo.CANID_SHOOTER_PITCH);

        configureFlywheel();
        configurePitchMotor();
        offsetPitchPos();

        pitchControlTaskObj = TrcTaskMgr.getInstance().createTask("PitchControlTask", this::pitchControlTask);
    }

    private void offsetPitchPos()
    {
        pitchMotor.motor.getSensorCollection().setPulseWidthPosition(0, 10);
        TrcUtil.sleep(50);
        int currPos = pitchMotor.motor.getSensorCollection().getPulseWidthPosition();
        int zeroPosTicks = TrcUtil.round(PITCH_OFFSET_TICKS + PITCH_OFFSET_DEG / PITCH_DEGREES_PER_COUNT);
        int low = (int) TrcUtil.modulo(zeroPosTicks, 4096);
        // minus is because sensor phase is inverted
        int high = (int) TrcUtil.modulo(low - TrcUtil.round(90 / PITCH_DEGREES_PER_COUNT), 4096);
        boolean crossZero = high > low; // this is because the sensor phase in inverted
        TrcDbgTrace.getGlobalTracer().traceInfo("Shooter.offsetPitchPos",
            "Zeroing Shooter: currPos=%d, zeroPos=%d, low=%d, high=%d, crossZero=%b", currPos, zeroPosTicks, low, high,
            crossZero);
        pitchMotor.motor.getSensorCollection().syncQuadratureWithPulseWidth(low, high, crossZero, -low, 10);
        TrcUtil.sleep(50);
        System.out.printf("Shooter pos=%.2f\n", getPitch());
        // TODO: remove the sleeps
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        robot.globalTracer.traceInfo("Shooter.setManualOverrideEnabled", "[%.3f] Shooter manual override enabled=%b",
            TrcUtil.getModeElapsedTime(), enabled);
        manualOverride = enabled;
    }

    public boolean isManualOverrideEnabled()
    {
        return manualOverride;
    }

    public void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            pitchControlTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            pitchControlTaskObj.unregisterTask();
        }
    }

    private void pitchControlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (pitchTicksTarget != null)
        {
            pitchMotor.motor.set(USE_MM ? ControlMode.MotionMagic : ControlMode.Position, pitchTicksTarget,
                DemandType.ArbitraryFeedForward, getPitchGravityComp());
            if (pitchEvent != null && !pitchEvent.isSignaled() && pitchOnTarget())
            {
                pitchEvent.set(true);
                pitchEvent = null;
            }
        }
    }

    public void stowShooter()
    {
        setPitch(RobotInfo.SHOOTER_BOTTOM_POS);
    }

    /**
     * Let the flywheel coast without applying power.
     */
    public void stopFlywheel()
    {
        flywheel.set(0.0);
    }

    public void setFlyWheelPower(double power)
    {
        flywheel.set(power);
    }

    /**
     * Set tangential velocity in inches per second
     *
     * @param velocity in/s
     */
    public void setFlywheelVelocity(double velocity)
    {
        flywheel.motor.set(ControlMode.Velocity, 0.1 * velocity / RobotInfo.FLYWHEEL_INCHES_PER_TICK);
    }

    public double getFlywheelVelocity()
    {
        return flywheel.getVelocity() * RobotInfo.FLYWHEEL_INCHES_PER_TICK;
    }

    public boolean pitchOnTarget()
    {
        return Math.abs(pitchMotor.motor.getClosedLoopError()) * PITCH_DEGREES_PER_COUNT <= PITCH_TOLERANCE;
    }

    public double getPitch()
    {
        return pitchMotor.getPosition() * PITCH_DEGREES_PER_COUNT;
    }

    public double getPitchVelocity()
    {
        return pitchMotor.getVelocity() * PITCH_DEGREES_PER_COUNT;
    }

    public void setPitch(double pitch)
    {
        setPitch(pitch, null);
    }

    public void setPitch(double pitch, TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        pitchEvent = event;
        pitchTicksTarget = TrcUtil.round(pitch / PITCH_DEGREES_PER_COUNT);
        setEnabled(true);
    }

    public void setPitchPower(double power)
    {
        if (manualOverride)
        {
            setEnabled(false);
            pitchMotor.set(power);
        }
    }

    private double getPitchGravityComp()
    {
        return 0; // theoretically, it's gravity neutral
    }

    private void configurePitchMotor()
    {
        pitchMotor.motor.config_kP(0, PITCH_kP, 10);
        pitchMotor.motor.config_kI(0, PITCH_kI, 10);
        pitchMotor.motor.config_kD(0, PITCH_kD, 10);
        pitchMotor.motor.config_kF(0, USE_MM ? PITCH_kF : 0, 10);
        pitchMotor.motor.config_IntegralZone(0, PITCH_IZONE, 10);
        if (USE_MM)
        {
            pitchMotor.motor.configMotionCruiseVelocity(PITCH_MAX_VEL, 10);
            pitchMotor.motor.configMotionAcceleration(PITCH_MAX_ACCEL, 10);
        }
        pitchMotor.motor.configAllowableClosedloopError(0, PITCH_ALLOWABLE_ERROR, 10);
        pitchMotor.motor.configClosedLoopPeakOutput(0, PITCH_MAX_POWER);
        pitchMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        pitchMotor.motor.enableVoltageCompensation(true);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        pitchMotor.motor.configClearPositionOnLimitR(false, 10);
        pitchMotor.setPositionSensorInverted(false);
        pitchMotor.setInverted(true);
        //        pitchMotor.motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        //        pitchMotor.motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        pitchMotor.configRevLimitSwitchNormallyOpen(false);
        pitchMotor.configFwdLimitSwitchNormallyOpen(false);
        pitchMotor.motor.overrideLimitSwitchesEnable(true);
        pitchMotor.motor.configForwardSoftLimitEnable(true, 10);
        pitchMotor.motor.configReverseSoftLimitEnable(true, 10);
        pitchMotor.motor.configForwardSoftLimitThreshold(PITCH_UPPER_LIMIT_TICKS, 10);
        pitchMotor.motor.configReverseSoftLimitThreshold(PITCH_LOWER_LIMIT_TICKS, 10);
    }

    private void configureFlywheel()
    {
        flywheel.motor.config_kP(0, FLYWHEEL_kP, 10);
        flywheel.motor.config_kI(0, FLYWHEEL_kI, 10);
        flywheel.motor.config_IntegralZone(0, FLYWHEEL_IZONE, 10);
        flywheel.motor.config_kD(0, FLYWHEEL_kD, 10);
        flywheel.motor.config_kF(0, FLYWHEEL_kF, 10);
        flywheel.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        flywheel.motor.enableVoltageCompensation(true);
        flywheel.setBrakeModeEnabled(false);
        flywheel.setInverted(true);
    }
}
