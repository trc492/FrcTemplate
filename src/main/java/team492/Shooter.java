package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ControlType;
import frclib.FrcCANSparkMax;
import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Shooter
{
    private static final double FLYWHEEL_kP = 0;
    private static final double FLYWHEEL_kI = 0;
    private static final double FLYWHEEL_kD = 0;
    private static final double FLYWHEEL_kF = 0;
    private static final double FLYWHEEL_IZONE = 0;
    private static final double FLYWHEEL_ROT_PER_TICK = 0;

    private static final double PITCH_kP = 0;
    private static final double PITCH_kI = 0;
    private static final double PITCH_kD = 0;
    private static final double PITCH_kF = 0;
    private static final int PITCH_IZONE = 0;
    private static final int PITCH_MAX_VEL = 0;
    private static final int PITCH_MAX_ACCEL = 0;
    private static final double PITCH_DEGREES_PER_COUNT = 0;
    private static final double PITCH_TOLERANCE = 0;

    private FrcCANSparkMax flywheel;
    private FrcCANTalon pitchMotor;
    private TrcTaskMgr.TaskObject pitchControlTaskObj;
    private TrcEvent pitchEvent;
    private int pitchTicksTarget;
    private boolean zeroCalibrating = false;
    private boolean zeroCalibrated = false;

    public Shooter()
    {
        flywheel = new FrcCANSparkMax("Shooter.flywheel", RobotInfo.CANID_FLYWHEEL, true);
        FrcCANSparkMax flywheelSlave = new FrcCANSparkMax("Shooter.flywheelSlave", RobotInfo.CANID_FLYWHEEL_SLAVE, true);
        pitchMotor = new FrcCANTalon("Shooter.pitchMotor", RobotInfo.CANID_SHOOTER_PITCH);

        configureFlywheel(flywheel);
        configureFlywheel(flywheelSlave);
        flywheelSlave.motor.follow(flywheel.motor);

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

        pitchControlTaskObj = TrcTaskMgr.getInstance().createTask("PitchControlTask", this::pitchControlTask);
    }

    public void init()
    {
        pitchControlTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        zeroCalibratePitch();
    }

    private void pitchControlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (zeroCalibrating)
        {
            if (pitchMotor.isLowerLimitSwitchActive()) {
                zeroCalibrating = false;
                zeroCalibrated = true;
                pitchMotor.set(0.0);
            }
            return;
        }
        if (!zeroCalibrated) return; // don't do anything if not calibrated
        pitchMotor.motor
            .set(ControlMode.MotionMagic, pitchTicksTarget, DemandType.ArbitraryFeedForward, getPitchGravityComp());
        if (pitchEvent != null && !pitchEvent.isSignaled() && pitchOnTarget())
        {
            pitchEvent.set(true);
        }
    }

    private void configureFlywheel(FrcCANSparkMax motor)
    {
        motor.motor.getPIDController().setP(FLYWHEEL_kP);
        motor.motor.getPIDController().setI(FLYWHEEL_kI);
        motor.motor.getPIDController().setD(FLYWHEEL_kD);
        motor.motor.getPIDController().setFF(FLYWHEEL_kF);
        motor.motor.getPIDController().setIZone(FLYWHEEL_IZONE);
        motor.motor.enableVoltageCompensation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.getEncoder().setPositionConversionFactor(FLYWHEEL_ROT_PER_TICK);
        motor.motor.getEncoder().setVelocityConversionFactor(FLYWHEEL_ROT_PER_TICK / 60.0);
        motor.setBrakeModeEnabled(false);
    }

    /**
     * Set rotation speed in rps
     *
     * @param velocity rps
     */
    public void setFlywheelVelocity(double velocity)
    {
        flywheel.motor.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public double getFlywheelVelocity()
    {
        return flywheel.motor.getEncoder().getVelocity();
    }

    public void zeroCalibratePitch()
    {
        zeroCalibrating = true;
        pitchMotor.set(-0.1); // limit switch will stop it and encoder will auto reset
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
        return 0;
    }
}
