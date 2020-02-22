package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import trclib.TrcDigitalInputTrigger;
import trclib.TrcEvent;
import trclib.TrcExclusiveSubsystem;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Conveyor implements TrcExclusiveSubsystem
{
    // TODO: tune this
    private static final double CONVEYOR_kP = 3.0;
    private static final double CONVEYOR_kI = 0;
    private static final double CONVEYOR_kD = 0;
    private static final double CONVEYOR_kF = 0;
    private static final int CONVEYOR_IZONE = 0;
    private static final double CONVEYOR_INCHES_PER_COUNT =
        (2 * Math.PI) / 4096.0 / 2.0; // divide by two bc it rolls 2x as fast
    private static final double CONVEYOR_TOLERANCE = 0.2;
    private static final double INTER_BALL_DISTANCE = 7.5; // inches

    private static final double SHOOT_POWER = 1.0;
    private static final double INTAKE_POWER = 0.5;

    public final FrcDigitalInput exitProximitySensor, entranceProximitySensor;
    private FrcCANTalon motor;
    private TrcTaskMgr.TaskObject advanceTask, intakeTask;
    private TrcEvent advanceEvent, shootEvent, intakeEvent;
    private boolean firstTrigger = false;
    private final TrcDigitalInputTrigger exitMonitor;
    private TrcDigitalInputTrigger shootTrigger;
    private int targetPosTicks = 0;
    private Robot robot;
    private boolean manualOverride;

    public Conveyor(Robot robot)
    {
        this.robot = robot;
        motor = new FrcCANTalon("Conveyor", RobotInfo.CANID_CONVEYOR);

        motor.motor.config_kP(0, CONVEYOR_kP, 10);
        motor.motor.config_kI(0, CONVEYOR_kI, 10);
        motor.motor.config_kD(0, CONVEYOR_kD, 10);
        motor.motor.config_kF(0, CONVEYOR_kF, 10);
        motor.motor.config_IntegralZone(0, CONVEYOR_IZONE, 10);
        motor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(true);
        motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        motor.setPositionSensorInverted(true);
        motor.setInverted(true);
        motor.configRevLimitSwitchNormallyOpen(true);
        motor.configFwdLimitSwitchNormallyOpen(true);
        motor.motor.overrideLimitSwitchesEnable(false);
        motor.resetPosition(true);

        exitProximitySensor = new FrcDigitalInput("ExitProximity", RobotInfo.CONVEYOR_PROXIMITY_SENSOR);
        exitProximitySensor.setInverted(true);
        entranceProximitySensor = new FrcDigitalInput("EntranceProximity", RobotInfo.INTAKE_PROXIMITY_SENSOR);
        entranceProximitySensor.setInverted(true);
        shootTrigger = new TrcDigitalInputTrigger("ShootTrigger", exitProximitySensor, this::shootTriggerEvent);
        exitMonitor = new TrcDigitalInputTrigger("ExitMonitorTrigger", exitProximitySensor, this::exitMonitorEvent);
        exitMonitor.setEnabled(true);

        advanceTask = TrcTaskMgr.getInstance().createTask("ConveyorAdvanceTask", this::advanceTask);
        intakeTask = TrcTaskMgr.getInstance().createTask("ConveyorIntakeTask", this::intakeTask);

        targetPosTicks = motor.motor.getSelectedSensorPosition();
    }

    private void intakeTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        boolean exitActive = exitProximitySensor.isActive();
        boolean entranceActive = entranceProximitySensor.isActive();
        if (exitActive || !entranceActive)
        {
            if (!exitActive)
            {
                robot.incNumBalls();
            }
            motor.set(0);
            if (intakeEvent != null)
            {
                intakeEvent.set(true);
                intakeEvent = null;
            }
            intakeTask.unregisterTask();
        }
        else
        {
            motor.set(INTAKE_POWER);
        }
    }

    private void exitMonitorEvent(boolean value)
    {
        if (!value && motor.getPower() >= 0)
        {
            robot.decNumBalls();
            if (robot.getNumBalls() < 0)
            {
                robot.globalTracer
                    .traceErr("Conveyor.exitMonitorEvent", "Invalid number of balls: %d! Resetting to zero.",
                        robot.getNumBalls());
                robot.setNumBalls(0);
            }
        }
    }

    private void shootTriggerEvent(boolean value)
    {
        robot.globalTracer.traceInfo("Conveyor.shootTriggerEvent", "Triggered! value=%b", value);
        if (firstTrigger && !value)
        {
            if (shootEvent != null)
            {
                shootEvent.set(true);
                shootEvent = null;
            }
            motor.set(0.0);
            shootTrigger.setEnabled(false);
        }
        firstTrigger = true;
    }

    public double getPosition()
    {
        return motor.getPosition() * CONVEYOR_INCHES_PER_COUNT;
    }

    public double getTargetPosition()
    {
        return targetPosTicks * CONVEYOR_INCHES_PER_COUNT;
    }

    public void setManualOverrideEnabled(boolean enabled)
    {
        manualOverride = enabled;
    }

    public boolean isManualOverrideEnabled()
    {
        return manualOverride;
    }

    public void setPower(double power)
    {
        setPower(null, power);
    }

    public void setPower(String owner, double power)
    {
        if (validateOwnership(owner))
        {
            stop(owner);
            motor.set(power);
        }
    }

    public void intake()
    {
        intake(null, null);
    }

    public void intake(String owner, TrcEvent event)
    {
        if (!manualOverride && validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }
            intakeEvent = event;

            intakeTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            shootTrigger.setEnabled(false);
            advanceTask.unregisterTask();

            motor.set(INTAKE_POWER);
        }
    }

    /**
     * Run the conveyor until a ball leaves the conveyor.
     */
    public void shoot()
    {
        shoot(null, null);
    }

    /**
     * Run the conveyor until a ball leaves the conveyor.
     *
     * @param owner The name of the routine calling this subsystem.
     *              If this isn't the owner, an exception will be thrown. To no-op instead, pass in null.
     * @param event The event to signal when the ball leaves.
     */
    public void shoot(String owner, TrcEvent event)
    {
        if (!manualOverride && validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }
            shootEvent = event;

            shootTrigger.setEnabled(true);
            advanceTask.unregisterTask();
            intakeTask.unregisterTask();
            firstTrigger = false;

            motor.set(SHOOT_POWER);
        }
    }

    /**
     * Advance the conveyor belt to allow for more intake.
     */
    public void advance()
    {
        advance(null, null, INTER_BALL_DISTANCE);
    }

    /**
     * Advance the conveyor belt to allow for more intake.
     *
     * @param owner The name of the routine calling this subsystem.
     *              If this isn't the owner, an exception will be thrown. To no-op instead, pass in null.
     * @param event The event to signal when done.
     * @param distance The distance to advance the conveyor, in inches
     */
    public void advance(String owner, TrcEvent event, double distance)
    {
        if (validateOwnership(owner)) // doesn't require checking for manual override
        {
            if (event != null)
            {
                event.clear();
            }
            this.advanceEvent = event;
            targetPosTicks = motor.motor.getSelectedSensorPosition();
            targetPosTicks += TrcUtil.round(distance / CONVEYOR_INCHES_PER_COUNT);
            motor.motor.set(ControlMode.Position, targetPosTicks);

            advanceTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            intakeTask.unregisterTask();
            shootTrigger.setEnabled(false);
        }
    }

    public void stop()
    {
        stop(null);
    }

    public void stop(String owner)
    {
        if (validateOwnership(owner))
        {
            motor.set(0.0);
            advanceTask.unregisterTask();
            intakeTask.unregisterTask();
            shootTrigger.setEnabled(false);
        }
    }

    private void advanceTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (onTarget())
        {
            if (advanceEvent != null)
            {
                advanceEvent.set(true);
                advanceEvent = null;
            }
            advanceTask.unregisterTask();
        }
    }

    public boolean onTarget()
    {
        return Math.abs(motor.motor.getClosedLoopError()) * CONVEYOR_INCHES_PER_COUNT <= CONVEYOR_TOLERANCE;
    }
}
