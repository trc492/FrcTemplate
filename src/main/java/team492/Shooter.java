package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ControlType;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import trclib.TrcAnalogInput;
import trclib.TrcAnalogSensor;
import trclib.TrcAnalogSensorTrigger;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Shooter
{
    // TODO: tune this
    private static final double FLYWHEEL_kP = 0.0008;
    private static final double FLYWHEEL_kI = 0.0;
    private static final double FLYWHEEL_IZONE = 10;
    private static final double FLYWHEEL_kD = 0.00025;
    private static final double FLYWHEEL_kD_THRESH_UPPER = 100; // in/s
    private static final double FLYWHEEL_kD_THRESH_LOWER = 80; // in/s
    private static final double FLYWHEEL_kF = 1.0 / RobotInfo.FLYWHEEL_TOP_SPEED;

    // TODO: tune this
    private static final boolean USE_MM = false;
    private static final double PITCH_kP = 0.65;
    private static final double PITCH_kI = 0;
    private static final double PITCH_kD = 0;
    private static final double PITCH_kF = 0;
    private static final double PITCH_MAX_POWER = 0.4;
    private static final int PITCH_IZONE = 0;
    private static final int PITCH_MAX_VEL = 0;
    private static final int PITCH_MAX_ACCEL = 0;
    private static final double PITCH_DEGREES_PER_COUNT = 360.0 / 4096.0 / 3.0; // 3:1 gear ratio, 4096 cpr
    private static final double PITCH_TOLERANCE = 1.5;

    // Measured
    private static final int PITCH_OFFSET_TICKS = 178;
    private static final double PITCH_OFFSET_DEG = 17.5;
    private static final int PITCH_UPPER_LIMIT_TICKS = 2700;
    private static final int PITCH_LOWER_LIMIT_TICKS = -25;

    private final TrcAnalogSensorTrigger<TrcAnalogInput.DataType> flywheelTrigger;

    public final FrcCANSparkMax flywheel;
    public final FrcCANTalon pitchMotor;
    private TrcTaskMgr.TaskObject pitchControlTaskObj;
    private TrcEvent pitchEvent;
    private int pitchTicksTarget;
    private double targetVelocity;
    private boolean manualOverride;

    public Shooter()
    {
        flywheel = new FrcCANSparkMax("Shooter.flywheel", RobotInfo.CANID_FLYWHEEL, true);
        pitchMotor = new FrcCANTalon("Shooter.pitchMotor", RobotInfo.CANID_SHOOTER_PITCH);

        configureFlywheel();
        configurePitchMotor();
        offsetPitchPos();

        TrcAnalogSensor flywheelError = new TrcAnalogSensor("Error",
            () -> Math.abs(getFlywheelVelocity() - targetVelocity));
        flywheelTrigger = new TrcAnalogSensorTrigger<>("", flywheelError, 0, TrcAnalogInput.DataType.RAW_DATA,
            new double[] { FLYWHEEL_kD_THRESH_LOWER }, this::flywheelErrorTrigger, false);

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

    private void flywheelErrorTrigger(int currZone, int prevZone, double value)
    {
        final String funcName = "flywheelErrorTrigger";
        TrcDbgTrace debug = TrcDbgTrace.getGlobalTracer();
        debug.traceInfo(funcName, "Edge event: curr=%d,prev=%d,val=%.2f", currZone, prevZone, value);
        if (value <= FLYWHEEL_kD_THRESH_LOWER)
        {
            debug.traceInfo(funcName, "Enabling D term!");
            flywheel.motor.getPIDController().setD(FLYWHEEL_kD);
            flywheelTrigger.setThresholds(new double[] { FLYWHEEL_kD_THRESH_UPPER });
        }
        else if (value > FLYWHEEL_kD_THRESH_UPPER)
        {
            debug.traceInfo(funcName, "Disabling D term!");
            flywheel.motor.getPIDController().setD(0.0);
            flywheelTrigger.setThresholds(new double[] { FLYWHEEL_kD_THRESH_LOWER });
        }
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
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
        flywheelTrigger.setEnabled(enabled);
    }

    private void pitchControlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        pitchMotor.motor
            .set(USE_MM ? ControlMode.MotionMagic : ControlMode.Position, pitchTicksTarget, DemandType.ArbitraryFeedForward, getPitchGravityComp());
        if (pitchEvent != null && !pitchEvent.isSignaled() && pitchOnTarget())
        {
            pitchEvent.set(true);
            pitchEvent = null;
        }
    }

    /**
     * Let the flywheel coast without applying power.
     */
    public void stopFlywheel()
    {
        targetVelocity = 0;
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
        targetVelocity = velocity;
        flywheel.motor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public double getFlywheelVelocity()
    {
        return flywheel.motor.getEncoder().getVelocity();
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
        pitchMotor.motor.config_kF(0, PITCH_kF, 10);
        pitchMotor.motor.config_IntegralZone(0, PITCH_IZONE, 10);
        if (USE_MM)
        {
            pitchMotor.motor.configMotionCruiseVelocity(PITCH_MAX_VEL, 10);
            pitchMotor.motor.configMotionAcceleration(PITCH_MAX_ACCEL, 10);
        }
        pitchMotor.motor.configClosedLoopPeakOutput(0, PITCH_MAX_POWER);
        pitchMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        pitchMotor.motor.enableVoltageCompensation(true);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        pitchMotor.motor.configClearPositionOnLimitR(false, 10);
        pitchMotor.setPositionSensorInverted(true);
        pitchMotor.setInverted(false);
        pitchMotor.configRevLimitSwitchNormallyOpen(false);
        pitchMotor.configFwdLimitSwitchNormallyOpen(false);
        pitchMotor.motor.overrideLimitSwitchesEnable(false);
        pitchMotor.motor.configForwardSoftLimitEnable(true, 10);
        pitchMotor.motor.configReverseSoftLimitEnable(true, 10);
        pitchMotor.motor.configForwardSoftLimitThreshold(PITCH_UPPER_LIMIT_TICKS, 10);
        pitchMotor.motor.configReverseSoftLimitThreshold(PITCH_LOWER_LIMIT_TICKS, 10);
    }

    private void configureFlywheel()
    {
        flywheel.motor.restoreFactoryDefaults();
        flywheel.motor.getPIDController().setP(FLYWHEEL_kP);
        flywheel.motor.getPIDController().setI(FLYWHEEL_kI);
        flywheel.motor.getPIDController().setIZone(FLYWHEEL_IZONE);
        flywheel.motor.getPIDController().setD(0.0); // kD is only enabled when close to target
        flywheel.motor.getPIDController().setFF(FLYWHEEL_kF);
        flywheel.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        flywheel.motor.getEncoder().setPositionConversionFactor(RobotInfo.FLYWHEEL_INCHES_PER_TICK);
        flywheel.motor.getEncoder().setVelocityConversionFactor(RobotInfo.FLYWHEEL_INCHES_PER_TICK / 60.0);
        flywheel.setBrakeModeEnabled(false);
        flywheel.setInverted(false);
        flywheel.motor.burnFlash(); // save to non-volatile memory, this way if the motor is reset the settings persist
    }
}
