package team492;

import frclib.FrcCANTalon;
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
    private Robot robot;

    private enum State
    {
        INTAKE, MONITOR, ADVANCE
    }

    private FrcCANTalon intakeMotor;
    private TrcTaskMgr.TaskObject intakeTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event, onFinishedEvent;
    private FrcPneumatic intakeExtension;
    private boolean singular = false;

    public Intake(Robot robot)
    {
        this.robot = robot;

        intakeMotor = new FrcCANTalon("Intake", RobotInfo.CANID_INTAKE);
        intakeMotor.setInverted(false);
        intakeMotor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        intakeMotor.motor.enableVoltageCompensation(true);

        intakeExtension = new FrcPneumatic("IntakeExtension", RobotInfo.CANID_PCM, RobotInfo.SOL_INTAKE_EXTEND,
            RobotInfo.SOL_INTAKE_RETRACT);

        sm = new TrcStateMachine<>("Intake.sm");
        event = new TrcEvent("Intake.event");

        intakeTaskObj = TrcTaskMgr.getInstance().createTask("IntakeTask", this::intakeTask);
    }

    public State getIntakeTaskState()
    {
        return sm.isEnabled() ? sm.getState() : null;
    }

    private void intakeTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case INTAKE:
                    event.clear();
                    intakeMotor.set(INTAKE_POWER);
                    // proximityTrigger.setEnabled(true);
                    extendIntake();
                    sm.setState(State.MONITOR);
                    break;

                case MONITOR:
                    if (robot.conveyor.entranceProximitySensor.isActive())
                    {
                        sm.setState(State.ADVANCE);
                    }
                    break;

                case ADVANCE:
                    robot.conveyor.intake(null, event);
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
                        sm.waitForSingleEvent(event, State.INTAKE); // reset state machine to prepare for another pickup
                    }
                    break;
            }
        }
    }

    public void stopIntake()
    {
        stopIntake(true);
    }

    public void stopIntake(boolean retract)
    {
        setIntakePower(0.0);
        if (retract)
        {
            retractIntake();
        }
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
