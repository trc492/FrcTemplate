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
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Shooter
{
    // TODO: tune this
    private static final double FLYWHEEL_kP = 0;
    private static final double FLYWHEEL_kI = 0;
    private static final double FLYWHEEL_IZONE = 0;
    private static final double FLYWHEEL_kD = 0;
    private static final double FLYWHEEL_kD_THRESH_UPPER = 20; // in/s
    private static final double FLYWHEEL_kD_THRESH_LOWER = 15; // in/s
    private static final double FLYWHEEL_kF = 0;
    private static final double FLYWHEEL_INCHES_PER_TICK = 0;

    // TODO: tune this
    private static final double PITCH_kP = 0;
    private static final double PITCH_kI = 0;
    private static final double PITCH_kD = 0;
    private static final double PITCH_kF = 0;
    private static final int PITCH_IZONE = 0;
    private static final int PITCH_MAX_VEL = 0;
    private static final int PITCH_MAX_ACCEL = 0;
    private static final double PITCH_DEGREES_PER_COUNT = 0;
    private static final double PITCH_TOLERANCE = 0;

    private static final double PITCH_CAL_POWER = -0.1;
    private final TrcAnalogSensorTrigger<TrcAnalogInput.DataType> flywheelTrigger;

    public FrcCANSparkMax flywheel;
    private FrcCANTalon pitchMotor;
    private TrcTaskMgr.TaskObject pitchControlTaskObj;
    private TrcEvent pitchEvent;
    private int pitchTicksTarget;
    private boolean zeroCalibrating = false;
    private boolean zeroCalibrated = false;
    private double targetVelocity;

    public Shooter()
    {
        flywheel = new FrcCANSparkMax("Shooter.flywheel", RobotInfo.CANID_FLYWHEEL, true);
        pitchMotor = new FrcCANTalon("Shooter.pitchMotor", RobotInfo.CANID_SHOOTER_PITCH);

        configureFlywheel(flywheel);

        pitchMotor.motor.config_kP(0, PITCH_kP, 10);
        pitchMotor.motor.config_kI(0, PITCH_kI, 10);
        pitchMotor.motor.config_kD(0, PITCH_kD, 10);
        pitchMotor.motor.config_kF(0, PITCH_kF, 10);
        pitchMotor.motor.config_IntegralZone(0, PITCH_IZONE, 10);
        pitchMotor.motor.configMotionCruiseVelocity(PITCH_MAX_VEL, 10);
        pitchMotor.motor.configMotionAcceleration(PITCH_MAX_ACCEL, 10);
        pitchMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        pitchMotor.motor.enableVoltageCompensation(true);
        pitchMotor.setBrakeModeEnabled(true);
        pitchMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        pitchMotor.motor.configClearPositionOnLimitR(true, 10);
        pitchMotor.setPositionSensorInverted(false);
        pitchMotor.setInverted(false);
        pitchMotor.configRevLimitSwitchNormallyOpen(true);
        pitchMotor.configFwdLimitSwitchNormallyOpen(true);
        pitchMotor.motor.overrideLimitSwitchesEnable(true);

        TrcAnalogSensor flywheelError = new TrcAnalogSensor("Error",
            () -> Math.abs(getFlywheelVelocity() - targetVelocity));
        flywheelTrigger = new TrcAnalogSensorTrigger<>("", flywheelError, 0, TrcAnalogInput.DataType.RAW_DATA,
            new double[] { FLYWHEEL_kD_THRESH_LOWER }, this::flywheelErrorTrigger, false);

        pitchControlTaskObj = TrcTaskMgr.getInstance().createTask("PitchControlTask", this::pitchControlTask);
    }

    private void flywheelErrorTrigger(int currZone, int prevZone, double value)
    {
        if (value <= FLYWHEEL_kD_THRESH_LOWER)
        {
            flywheel.motor.getPIDController().setD(FLYWHEEL_kD);
            flywheelTrigger.setThresholds(new double[] { FLYWHEEL_kD_THRESH_UPPER });
        }
        else if (value > FLYWHEEL_kD_THRESH_UPPER)
        {
            flywheel.motor.getPIDController().setD(0.0);
            flywheelTrigger.setThresholds(new double[] { FLYWHEEL_kD_THRESH_LOWER });
        }
    }

    public void init()
    {
        pitchControlTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        flywheelTrigger.setEnabled(true);
        zeroCalibratePitch();
    }

    private void pitchControlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (zeroCalibrating)
        {
            if (pitchMotor.isLowerLimitSwitchActive())
            {
                zeroCalibrating = false;
                zeroCalibrated = true;
                pitchMotor.set(0.0);
            }
            return;
        }
        if (!zeroCalibrated)
            return; // don't do anything if not calibrated
        pitchMotor.motor
            .set(ControlMode.MotionMagic, pitchTicksTarget, DemandType.ArbitraryFeedForward, getPitchGravityComp());
        if (pitchEvent != null && !pitchEvent.isSignaled() && pitchOnTarget())
        {
            pitchEvent.set(true);
            pitchEvent = null;
        }
    }

    private void configureFlywheel(FrcCANSparkMax motor)
    {
        motor.motor.getPIDController().setP(FLYWHEEL_kP);
        motor.motor.getPIDController().setI(FLYWHEEL_kI);
        motor.motor.getPIDController().setIZone(FLYWHEEL_IZONE);
        motor.motor.getPIDController().setD(0.0); // kD is only enabled when close to target
        motor.motor.getPIDController().setFF(FLYWHEEL_kF);
        motor.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.getEncoder().setPositionConversionFactor(FLYWHEEL_INCHES_PER_TICK);
        motor.motor.getEncoder().setVelocityConversionFactor(FLYWHEEL_INCHES_PER_TICK / 60.0);
        motor.setBrakeModeEnabled(false);
    }

    /**
     * Let the flywheel coast without applying power.
     */
    public void stopFlywheel()
    {
        targetVelocity = 0;
        flywheel.set(0.0);
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

    public void zeroCalibratePitch()
    {
        zeroCalibrating = true;
        pitchMotor.set(PITCH_CAL_POWER); // limit switch will stop it and encoder will auto reset
    }

    public boolean pitchOnTarget()
    {
        return Math.abs(pitchMotor.motor.getClosedLoopError()) * PITCH_DEGREES_PER_COUNT <= PITCH_TOLERANCE;
    }

    public double getPitch()
    {
        return pitchMotor.getPosition() * PITCH_DEGREES_PER_COUNT;
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
    }

    private double getPitchGravityComp()
    {
        return 0; // TODO: tune this
    }
}
