package team492;

import TrcCommonLib.trclib.TrcDigitalInputTrigger;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcFrcLib.frclib.FrcDigitalInput;
import TrcFrcLib.frclib.FrcPWMTalonSRX;

public class Conveyor implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Conveyor";

    private final Robot robot;
    private final FrcPWMTalonSRX horizontalConveyor, verticalConveyor;
    private final FrcDigitalInput beamBreak;
    private final TrcDigitalInputTrigger beamBreakTrigger;

    public Conveyor(Robot robot)
    {
        this.robot = robot;
        horizontalConveyor = new FrcPWMTalonSRX(
            "horizontalConveyor", RobotParams.PWM_CHANNEL_HORIZONTAL_CONVEYOR, null, null, null);
        horizontalConveyor.setInverted(true);
        verticalConveyor = new FrcPWMTalonSRX(
            "verticalConveyor", RobotParams.PWM_CHANNEL_VERTICAL_CONVEYOR, null, null, null);
        beamBreak = new FrcDigitalInput("beamBreaker", RobotParams.DIO_BEAM_BREAKER);
        beamBreakTrigger = new TrcDigitalInputTrigger("beamBreakerTrigger", beamBreak, this::beamBreakEvent);
    }

    public void setPower(double horizontalPower, double verticalPower)
    {
        horizontalConveyor.set(horizontalPower);
        verticalConveyor.set(verticalPower);
    }

    public boolean isBeamBreakActive()
    {
        return beamBreak.isActive();
    }

    private void beamBreakEvent(boolean active)
    {
        if(active)
        {
            robot.dashboard.displayPrintf(11, "Conveyor: beamBreakEvent triggered");
            robot.intake.stop();
            horizontalConveyor.stopMotor();
            verticalConveyor.stopMotor();
        }
    }
}
