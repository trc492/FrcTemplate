package team492;

import frclib.FrcCANTalon;
import frclib.FrcPneumatic;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

import java.util.Arrays;

public class Intake
{
    private static final double DEF_INTAKE_POWER = 0.5;

    private enum State
    {
        BACKUP, FORWARD, INTAKE, MONITOR, ADVANCE, SECURE
    }

    private Robot robot;
    private double intakePower;
    private double conveyorPower;
    private boolean extend;
    private FrcCANTalon intakeMotor;
    private TrcTaskMgr.TaskObject intakeTaskObj;
    private TrcStateMachine<State> sm;
    private TrcEvent event, onFinishedEvent;
    private FrcPneumatic intakeExtension;
    private boolean singular = false;
    private Double conveyorIntakeStartPos = null;
    private double spacingDistance = 4;

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

        HalDashboard.putNumber("IntakeState", -1);
        HalDashboard.putNumber("EntranceSensor", 0);
    }

    public void setSpacingDistance(double distance)
    {
        this.spacingDistance = distance;
    }

    private int stateIndex()
    {
        return sm.getState() == null ? -1 : Arrays.asList(State.values()).indexOf(sm.getState());
    }

    public State getIntakeTaskState()
    {
        return sm.isEnabled() ? sm.getState() : null;
    }

    private void intakeTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        HalDashboard.putNumber("IntakeState", stateIndex());
        HalDashboard.putNumber("EntranceSensor", robot.conveyor.entranceProximitySensor.isActive() ? 1 : 0);
        if (state != null)
        {
            switch (state)
            {
                case BACKUP:
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
                    robot.conveyor.advance(null, event, spacingDistance);
                    sm.setState(State.INTAKE);
                    break;

                case INTAKE:
                    if (!robot.conveyor.isManualOverrideEnabled() && robot.conveyor.exitProximitySensor.isActive())
                    {
                        stopIntake();
                    }
                    else
                    {
                        event.clear();
                        intakeMotor.set(intakePower);
                        if (extend)
                        {
                            extendIntake();
                        }
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
                    robot.conveyor.intake(null, event, conveyorPower);
                    sm.waitForSingleEvent(event, State.SECURE);
                    break;

                case SECURE:
                    event.clear();
                    robot.conveyor.stop();
                    robot.conveyor.advance(null, event, spacingDistance);
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
        if (isActive())
        {
            robot.conveyor.stop();
        }
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
        intakeMultiple(true, DEF_INTAKE_POWER, Conveyor.DEF_INTAKE_POWER);
    }

    public void intakeMultiple(boolean extend)
    {
        intakeMultiple(extend, DEF_INTAKE_POWER, Conveyor.DEF_INTAKE_POWER);
    }

    public void intakeMultiple(boolean extend, double intakePower, double conveyorPower)
    {
        intake(false, extend, null, intakePower, conveyorPower);
    }

    public void intakeOnce()
    {
        intakeOnce(null);
    }

    public void intakeOnce(TrcEvent onFinishedEvent)
    {
        intake(true, true, onFinishedEvent, DEF_INTAKE_POWER, Conveyor.DEF_INTAKE_POWER);
    }

    private void intake(boolean singular, boolean extend, TrcEvent onFinishedEvent, double intakePower, double conveyorPower)
    {
        if (isActive())
        {
            return;
        }
        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.extend = extend;
        this.singular = singular;
        this.onFinishedEvent = onFinishedEvent;
        this.intakePower = intakePower;
        this.conveyorPower = conveyorPower;
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
