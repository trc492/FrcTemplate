package team492;

import frclib.FrcCANFalcon;
import frclib.FrcJoystick;
import frclib.FrcPneumatic;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Climber
{
    private enum State
    {
        INIT, UNLATCH, CLIMB
    }

    private static final double SHOOTER_CLIMB_POS = 60;
    private static final double CLIMB_POWER = 0.8;

    private Robot robot;
    private TrcStateMachine<State> sm;
    private TrcEvent event;
    private TrcTaskMgr.TaskObject climberTaskObj;
    private FrcPneumatic latch;
    private FrcCANFalcon motor;

    public Climber(Robot robot)
    {
        this.robot = robot;

        sm = new TrcStateMachine<>("Climber.sm");
        event = new TrcEvent("Climber.event");
        climberTaskObj = TrcTaskMgr.getInstance().createTask("Climber.climberTask", this::climberTask);

        latch = new FrcPneumatic("Climber.latch", RobotInfo.CANID_PCM, RobotInfo.SOL_LATCH_EXTEND,
            RobotInfo.SOL_LATCH_RETRACT);

        motor = new FrcCANFalcon("Climber.motor", RobotInfo.CANID_CLIMBER);
        motor.setInverted(false);
        motor.setBrakeModeEnabled(true);
        motor.motor.configOpenloopRamp(1.0, 10);
        motor.motor.configVoltageCompSaturation(RobotInfo.BATTERY_NOMINAL_VOLTAGE);
        motor.motor.enableVoltageCompensation(true);
    }

    public void setPower(double power)
    {
        motor.set(power);
    }

    public void unlatch()
    {
        latch.extend(); // TODO: depends on mechanism. Maybe swap?
    }

    public void latch()
    {
        latch.retract();
    }

    public boolean isActive()
    {
        return climberTaskObj.isRegistered();
    }

    public void cancel()
    {
        motor.set(0);
        climberTaskObj.unregisterTask();
    }

    private void climberTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            switch (state)
            {
                case INIT:
                    robot.shooter.setPitch(SHOOTER_CLIMB_POS, event);
                    sm.waitForSingleEvent(event, State.UNLATCH);
                    break;

                case UNLATCH:
                    unlatch();
                    robot.shooter.setManualOverrideEnabled(true);
                    sm.setState(State.CLIMB);
                    break;

                case CLIMB:
                    if (robot.buttonPanel.getRawButton(FrcJoystick.PANEL_BUTTON_BLUE1))
                    {
                        motor.set(CLIMB_POWER);
                    }
                    else
                    {
                        motor.set(0);
                    }
                    break;
            }
        }
    }

    public void startClimber()
    {
        climberTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        sm.start(State.INIT);
    }
}
