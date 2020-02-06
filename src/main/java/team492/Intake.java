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
        INTAKE, ADVANCE
    }

    private FrcCANTalon intakeMotor;
    private TrcTaskMgr.TaskObject intakeTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event, onFinishedEvent;
    private TrcDigitalInputTrigger proximityTrigger;
    private TrcTimer timer;
    private FrcPneumatic intakeExtension;
    private boolean singular = false;

    public Intake(Robot robot)
    {
        this.robot = robot;

        proximityTrigger = new TrcDigitalInputTrigger("Intake.trigger", robot.conveyor.entranceProximitySensor, this::proximityTriggerEvent);

        intakeMotor = new FrcCANTalon("Intake", RobotInfo.CANID_INTAKE);
        intakeMotor.setInverted(false);
        intakeMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        intakeMotor.motor.enableVoltageCompensation(true);

        intakeExtension = new FrcPneumatic("IntakeExtension", RobotInfo.CANID_PCM, RobotInfo.SOL_INTAKE_EXTEND,
            RobotInfo.SOL_INTAKE_RETRACT);

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
                case INTAKE:
                    event.clear();
                    setIntakePower(INTAKE_POWER);
                    proximityTrigger.setEnabled(true);
                    extendIntake();
                    sm.waitForSingleEvent(event, State.ADVANCE);
                    break;

                case ADVANCE:
                    robot.conveyor.intake();
                    robot.incNumBalls();
                    if (onFinishedEvent != null)
                    {
                        onFinishedEvent.set(true);
                        onFinishedEvent = null;
                    }
                    if (singular)
                    {
                        stopIntake(); // if only picking up one, stop now
                    }
                    else
                    {
                        sm.setState(State.INTAKE); // reset state machine to prepare for another pickup
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

    public void intakeMultiple()
    {
        intake(false, null);
    }

    public void intakeOnce()
    {
        intakeOnce(null);
    }

    public void intakeOnce(TrcEvent onFinishedEvent)
    {
        intake(true, onFinishedEvent);
    }

    private void intake(boolean singular, TrcEvent onFinishedEvent)
    {
        if (isActive())
        {
            return;
        }
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.singular = singular;
        this.onFinishedEvent = onFinishedEvent;
        event.clear();
        intakeTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        sm.start(State.INTAKE);
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
