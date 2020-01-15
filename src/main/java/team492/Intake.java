package team492;

import frclib.FrcCANTalon;
import frclib.FrcDigitalInput;
import frclib.FrcPneumatic;
import trclib.TrcDigitalInputTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

public class Intake
{
    private static final double INTAKE_POWER = 1.0;
    private static final double INTAKE_DELAY = 0.1;
    private Robot robot;

    private enum State
    {
        Intake, Wait, Advance
    }

    private FrcDigitalInput proximitySensor;
    private FrcCANTalon intakeMotor;
    private TrcTaskMgr.TaskObject intakeTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event, onFinishedEvent;
    private TrcDigitalInputTrigger proximityTrigger;
    private TrcTimer timer;
    private FrcPneumatic intakeExtension;

    public Intake(Robot robot)
    {
        this.robot = robot;
        proximitySensor = new FrcDigitalInput("IntakeProximity", RobotInfo.INTAKE_PROXIMITY_SENSOR);

        proximityTrigger = new TrcDigitalInputTrigger("Intake.trigger", proximitySensor, this::proximityTriggerEvent);

        intakeMotor = new FrcCANTalon("Intake", RobotInfo.CANID_INTAKE);
        intakeMotor.setInverted(false);
        intakeMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        intakeMotor.motor.enableVoltageCompensation(true);

        intakeExtension = new FrcPneumatic("IntakeExtension", RobotInfo.CANID_PCM, RobotInfo.SOL_INTAKE_EXTEND, RobotInfo.SOL_INTAKE_RETRACT);

        sm = new TrcStateMachine<>("Intake.sm");
        event = new TrcEvent("Intake.event");

        intakeTaskObj = TrcTaskMgr.getInstance().createTask("IntakeTask", this::intakeTask);

        timer = new TrcTimer("Intake.timer");
    }

    private void proximityTriggerEvent(boolean active)
    {
        if (active)
        {
            event.set(true);
            proximityTrigger.setEnabled(false);
        }
    }

    private void intakeTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.getState();
        if (state != null)
        {
            switch (state)
            {
                case Intake:
                    setIntakePower(INTAKE_POWER);
                    proximityTrigger.setEnabled(true);
                    extendIntake();
                    sm.waitForSingleEvent(event, State.Wait);
                    break;

                case Wait:
                    timer.set(INTAKE_DELAY, event);
                    sm.waitForSingleEvent(event, State.Advance);
                    break;

                case Advance:
                    setIntakePower(0.0);
                    // TODO: Start another intake operation?
                    robot.conveyor.advance();
                    robot.numBalls++;
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                    }
                    break;
            }
        }
    }

    public void stopIntake()
    {
        setIntakePower(0.0);
        retractIntake();
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    public void intake()
    {
        intake(null);
    }

    public void intake(TrcEvent onFinishedEvent)
    {
        if (isActive())
        {
            return;
        }
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;
        event.clear();
        intakeTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        sm.start(State.Intake);
    }

    public void extendIntake()
    {
        intakeExtension.extend();
    }

    public void retractIntake()
    {
        intakeExtension.retract();
    }

    public void setIntakePower(double power)
    {
        if (isActive())
        {
            sm.stop();
            intakeTaskObj.unregisterTask();
        }
        intakeMotor.set(power);
    }
}
