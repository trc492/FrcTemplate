package team492;

import java.util.concurrent.atomic.AtomicBoolean;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPWMTalonSRX;

public class Conveyor implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Conveyor";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

    private final Robot robot;
    private final FrcPWMTalonSRX horizontalConveyor, verticalConveyor;
    private final FrcDigitalInput beamBreak;
    private final TrcEvent beamBreakEvent;
    private final TrcDigitalInputTrigger beamBreakTrigger;

    private String currOwner;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject intakeTaskObj;

    public Conveyor(Robot robot)
    {
        this.robot = robot;
        horizontalConveyor = new FrcPWMTalonSRX(
            "horizontalConveyor", RobotParams.PWM_CHANNEL_HORIZONTAL_CONVEYOR, null, null, null);
        horizontalConveyor.setInverted(true);
        verticalConveyor = new FrcPWMTalonSRX(
            "verticalConveyor", RobotParams.PWM_CHANNEL_VERTICAL_CONVEYOR, null, null, null);
        verticalConveyor.setInverted(true);
        
        beamBreak = new FrcDigitalInput("beamBreaker", RobotParams.DIO_BEAM_BREAKER);
        beamBreak.setInverted(true);
        beamBreakEvent = new TrcEvent("beamBreakEvent");
        beamBreakTrigger = new TrcDigitalInputTrigger("beamBreakerTrigger", beamBreak, this::beamBreakEvent);
        beamBreakTrigger.setEnabled(true);

        event = new TrcEvent("event");
        sm = new TrcStateMachine<>(moduleName);
        intakeTaskObj = TrcTaskMgr.createTask(moduleName + ".intakeTask", this::autoIntakeTask);
    }

    public void setPower(double horizontalPower, double verticalPower)
    {
        horizontalConveyor.set(horizontalPower);
        verticalConveyor.set(verticalPower);
    }

    public void setHorizontalPower(double power)
    {
        horizontalConveyor.set(power);
    }

    public void setVerticalPower(double power)
    {
        verticalConveyor.set(power);
    }

    public boolean isBeamBreakBroken()
    {
        return beamBreak.isActive();
    }

    private enum State
    {
        START,
        ADJUST_BALL,
        LIFT,
        DONE
    }

    private void beamBreakEvent(Object obj)
    {
        if(obj instanceof AtomicBoolean)
        {
            if(((AtomicBoolean) obj).get())
            {
                beamBreakEvent.signal();
            }
        }
    }

    public boolean intakeAndAdvance(String owner)
    {
        boolean success = false;
        if(this.acquireExclusiveAccess(owner)
            && this.horizontalConveyor.acquireExclusiveAccess(owner)
            && this.verticalConveyor.acquireExclusiveAccess(owner)
            && robot.intake.acquireExclusiveAccess(owner))
        {
            currOwner = owner;
            sm.start(State.START);
            intakeTaskObj.registerTask(TaskType.FAST_POSTPERIODIC_TASK);
            success = true;
        }
        globalTracer.traceInfo(

            "intakeAndAdvance", "[%.3f] owner=%s, success=%s",
            TrcUtil.getModeElapsedTime(), owner, success);
        return success;
    }

    private void autoIntakeTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if(state != null)
        {
            double matchTime = TrcUtil.getModeElapsedTime();

            switch(state)
            {
                case START:
                    robot.intake.setPower(currOwner, 0.0, RobotParams.INTAKE_PICKUP_POWER, 0.0);
                    horizontalConveyor.set(currOwner, 0.0, RobotParams.HORIZONTAL_CONVEYOR_POWER, 0.0, null);
                    sm.waitForSingleEvent(beamBreakEvent, State.ADJUST_BALL);
                    break;

                case ADJUST_BALL:
                    robot.intake.setPower(currOwner, 0.0, 0.0, 0.0);
                    horizontalConveyor.set(currOwner, 0.0, RobotParams.HORIZONTAL_CONVEYOR_ADJUST_POWER, 0.2, event);
                    sm.waitForSingleEvent(event, State.LIFT);
                    break;

                case LIFT:
                    horizontalConveyor.set(currOwner, 0.0, 0.0, 0.0, null);
                    verticalConveyor.set(currOwner, 0.0, RobotParams.VERTICAL_CONVEYOR_POWER, 0.35, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    break;
            }
        }
    }

    public void cancel()
    {
        robot.intake.stop(currOwner, 0);
        if(currOwner != null)
        {
            this.releaseExclusiveAccess(currOwner);
            this.horizontalConveyor.releaseExclusiveAccess(currOwner);
            this.verticalConveyor.releaseExclusiveAccess(currOwner);
            robot.intake.releaseExclusiveAccess(currOwner);
            currOwner = null;
        }
        sm.stop();
        intakeTaskObj.unregisterTask();
    }

    public double getHorizontalConveyorPower()
    {
        return horizontalConveyor.getMotorPower();
    }

    public double getVerticalConveyorPower()
    {
        return verticalConveyor.getMotorPower();
    }
}