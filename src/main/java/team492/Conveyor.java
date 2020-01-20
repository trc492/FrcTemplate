package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import trclib.TrcDigitalInputTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

public class Conveyor
{
    // TODO: tune this
    private static final double CONVEYOR_kP = 0;
    private static final double CONVEYOR_kI = 0;
    private static final double CONVEYOR_kD = 0;
    private static final double CONVEYOR_kF = 0;
    private static final int CONVEYOR_IZONE = 0;
    private static final int CONVEYOR_MAX_VEL = 0;
    private static final int CONVEYOR_MAX_ACCEL = 0;
    private static final double CONVEYOR_INCHES_PER_COUNT = 0;
    private static final double CONVEYOR_TOLERANCE = 0;
    private static final double INTER_BALL_DISTANCE = 7.5; // inches

    private static final double READY_POWER = 0.4;
    private static final double SHOOT_POWER = 0.7;

    private FrcCANTalon motor;
    private TrcTaskMgr.TaskObject advanceTask;
    private TrcEvent advanceEvent, readyEvent, shootEvent;
    private FrcDigitalInput proximitySensor;
    private final TrcDigitalInputTrigger exitMonitor;
    private TrcDigitalInputTrigger readyTrigger, shootTrigger;
    private int targetPosTicks = 0;
    private Robot robot;

    public Conveyor(Robot robot)
    {
        this.robot = robot;
        motor = new FrcCANTalon("Conveyor", RobotInfo.CANID_CONVEYOR);

        motor.motor.config_kP(0, CONVEYOR_kP, 10);
        motor.motor.config_kI(0, CONVEYOR_kI, 10);
        motor.motor.config_kD(0, CONVEYOR_kD, 10);
        motor.motor.config_kF(0, CONVEYOR_kF, 10);
        motor.motor.config_IntegralZone(0, CONVEYOR_IZONE, 10);
        motor.motor.configMotionCruiseVelocity(CONVEYOR_MAX_VEL, 10);
        motor.motor.configMotionAcceleration(CONVEYOR_MAX_ACCEL, 10);
        motor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
        motor.setBrakeModeEnabled(true);
        motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        motor.setPositionSensorInverted(false);
        motor.setInverted(false);
        motor.configRevLimitSwitchNormallyOpen(true);
        motor.configFwdLimitSwitchNormallyOpen(true);
        motor.motor.overrideLimitSwitchesEnable(false);
        motor.resetPosition(true);

        proximitySensor = new FrcDigitalInput("Proximity", RobotInfo.CONVEYOR_PROXIMITY_SENSOR);
        readyTrigger = new TrcDigitalInputTrigger("ReadyTrigger", proximitySensor, this::readyTriggerEvent);
        shootTrigger = new TrcDigitalInputTrigger("ShootTrigger", proximitySensor, this::shootTriggerEvent);
        exitMonitor = new TrcDigitalInputTrigger("ExitMonitorTrigger", proximitySensor, this::exitMonitorEvent);
        exitMonitor.setEnabled(true);

        advanceTask = TrcTaskMgr.getInstance().createTask("ConveyorAdvanceTask", this::advanceTask);

        targetPosTicks = motor.motor.getSelectedSensorPosition();
    }

    private void exitMonitorEvent(boolean value)
    {
        if (!value)
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
        if (!value)
        {
            if (shootEvent != null)
            {
                shootEvent.set(true);
                shootEvent = null;
            }
            motor.set(0.0);
            shootTrigger.setEnabled(false);
        }
    }

    private void readyTriggerEvent(boolean value)
    {
        if (value)
        {
            if (readyEvent != null)
            {
                readyEvent.set(true);
                readyEvent = null;
            }
            motor.set(0.0);
            readyTrigger.setEnabled(false);
        }
    }

    /**
     * Run the conveyor until a ball leaves the conveyor. The ball doesn't have to be readied.
     */
    public void shoot()
    {
        shoot(null);
    }

    /**
     * Run the conveyor until a ball leaves the conveyor.  The ball doesn't have to be readied.
     *
     * @param event The event to signal when the ball leaves.
     */
    public void shoot(TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        shootEvent = event;

        shootTrigger.setEnabled(true);
        readyTrigger.setEnabled(false);
        advanceTask.unregisterTask();

        motor.set(SHOOT_POWER);
    }

    /**
     * Run the conveyor until a ball is on the edge of the conveyor.
     */
    public void readyShot()
    {
        readyShot(null);
    }

    /**
     * Run the conveyor until a ball is on the edge of the conveyor.
     *
     * @param event The event to signal when done.
     */
    public void readyShot(TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }

        if (proximitySensor.isActive())
        {
            if (event != null)
            {
                event.set(true);
            }
        }
        else
        {
            this.readyEvent = event;
            shootTrigger.setEnabled(false);
            readyTrigger.setEnabled(true);
            advanceTask.unregisterTask();

            motor.set(READY_POWER);
        }
    }

    /**
     * Advance the conveyor belt to allow for more intake.
     */
    public void advance()
    {
        advance(null);
    }

    /**
     * Advance the conveyor belt to allow for more intake.
     *
     * @param event The event to signal when done.
     */
    public void advance(TrcEvent event)
    {
        if (event != null)
        {
            event.clear();
        }
        this.advanceEvent = event;
        targetPosTicks += TrcUtil.round(INTER_BALL_DISTANCE / CONVEYOR_INCHES_PER_COUNT);
        motor.motor.set(ControlMode.MotionMagic, targetPosTicks);

        advanceTask.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        readyTrigger.setEnabled(false);
        shootTrigger.setEnabled(false);
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
