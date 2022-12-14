package team492;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcUtil;
import TrcFrcLib.frclib.FrcPWMTalonSRX;
import TrcFrcLib.frclib.FrcPneumatic;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake implements TrcExclusiveSubsystem
{
    private static final String moduleName = "Intake";

    private final Robot robot;
    private final FrcPWMTalonSRX intakeMotorController;
    // private final FrcPneumatic intakePneumatic;
    private TrcEvent onFinishedEvent;
    private TrcDbgTrace msgTracer = null;

    public Intake(Robot robot)
    {
        this.robot = robot;
        intakeMotorController = new FrcPWMTalonSRX(moduleName + ".motorController",
            RobotParams.PWM_CHANNEL_INTAKE, null, null, null);
        intakeMotorController.setInverted(true);
        // intakePneumatic = new FrcPneumatic(
        //     moduleName + ".pneumatic", RobotParams.CANID_PCM, PneumaticsModuleType.CTREPCM,
        //     RobotParams.PNEUMATIC_INTAKE_RETRACT, RobotParams.PNEUMATIC_INTAKE_EXTEND);
    }

    /**
     * This method enables/disables tracing for the shooter subsystem.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    public void setPower(String owner, double delay, double power, double duration)
    {
        final String funcName = "setPower";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(
                funcName, "[%.3f] owner=%s, delay=%.1f, power=%.1f, duration=%.3f",
                TrcUtil.getModeElapsedTime(), owner, delay, power, duration);
        }

        if (validateOwnership(owner))
        {
            intakeMotorController.set(delay, power, duration);
        }
    }   //setPower

    public void setPower(double delay, double power, double duration)
    {
        setPower(null, delay, power, duration);
    }   //setPower

    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0);
    }   //setPower

    public void pickup(String owner, TrcEvent event)
    {
        if(validateOwnership(owner))
        {
            this.onFinishedEvent = event;
        }
    }

    public void stop(String owner, double delay)
    {
        final String funcName = "stop";

        if (msgTracer != null)
        {
            msgTracer.traceInfo(funcName, "[%.3f] owner=%s", TrcUtil.getModeElapsedTime(), owner);
        }

        if (validateOwnership(owner))
        {
            setPower(delay, 0.0, 0.0);
        }
    }   //stop

    public void stop(double delay)
    {
        stop(null, delay);
    }   //stop

    public void stop()
    {
        stop(null, 0.0);
    }   //stop
}
