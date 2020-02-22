package team492;

import frclib.FrcCANTalon;
import frclib.FrcPneumatic;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Intake
{
    private static final double INTAKE_POWER = 0.5;
    private Robot robot;

    private enum State
    {
        BACKUP, FORWARD, INTAKE, MONITOR, ADVANCE, SECURE
    }

    private FrcCANTalon intakeMotor;
    private TrcTaskMgr.TaskObject intakeTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event, onFinishedEvent;
    private FrcPneumatic intakeExtension;
    private boolean singular = false;
    private Double conveyorIntakeStartPos = null;

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
                case BACKUP:
//                    if (robot.conveyor.readyForPickup)
//                    {
//                        sm.setState(State.INTAKE);
//                        break;
//                    }
                    if (conveyorIntakeStartPos == null)
                    {
                        conveyorIntakeStartPos = robot.conveyor.getPosition();
                    }
                    double distMoved = robot.conveyor.getPosition() - conveyorIntakeStartPos;
                    if (distMoved > -50 && !robot.conveyor.entranceProximitySensor.isActive())
                    {
                        robot.conveyor.setPower(-1.0);
                    }
                    else
                    {
                        robot.conveyor.setPower(0);
                        conveyorIntakeStartPos = null;
                        sm.setState(State.FORWARD);
                    }
                    break;

                case FORWARD:
                    robot.conveyor.advance(null, event, 4);
                    sm.setState(State.INTAKE);
//                    if (robot.conveyor.entranceProximitySensor.isActive())
//                    {
//                        robot.conveyor.setPower(1.0);
//                    }
//                    else
//                    {
//                        robot.conveyor.setPower(0);
//                        sm.setState(State.INTAKE);
//                    }
                    break;

                case INTAKE:
                    if (!robot.conveyor.isManualOverrideEnabled() && robot.conveyor.exitProximitySensor.isActive())
                    {
                        stopIntake();
                    }
                    else
                    {
                        event.clear();
                        intakeMotor.set(INTAKE_POWER);
                        extendIntake();
                        sm.setState(State.MONITOR);
                    }
                    break;

                case MONITOR:
                    if (robot.conveyor.entranceProximitySensor.isActive())
                    {
                        sm.setState(State.ADVANCE);
                    }
                    break;

                case ADVANCE:
                    robot.conveyor.intake(null, event);
                    sm.waitForSingleEvent(event, State.SECURE);
                    break;

                case SECURE:
                    event.clear();
                    robot.conveyor.stop();
                    robot.conveyor.advance(null, event, 4);
                    robot.conveyor.readyForPickup = true;
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
        conveyorIntakeStartPos = null;
        event.clear();
        intakeTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        sm.start(robot.conveyor.isManualOverrideEnabled() ? State.INTAKE : State.BACKUP);
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
