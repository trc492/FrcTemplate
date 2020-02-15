package team492;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frclib.FrcCANTalon;
import trclib.TrcRobot;

import java.util.function.DoubleSupplier;

/**
 * For use with the FRC Characterization tool
 */
public class CmdTalonCharacterization implements TrcRobot.RobotCommand
{
    private final DoubleSupplier encoderPosition;
    private final DoubleSupplier encoderRate;
    private FrcCANTalon talon;
    private NetworkTableEntry autoSpeedEntry;
    private NetworkTableEntry telemetryEntry;
    private Number[] numberArray = new Number[6];

    public CmdTalonCharacterization(DoubleSupplier encoderPosition, DoubleSupplier encoderRate, FrcCANTalon talon)
    {
        this.encoderPosition = encoderPosition;
        this.encoderRate = encoderRate;
        this.talon = talon;

        autoSpeedEntry =
            NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
        telemetryEntry =
            NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double position = encoderPosition.getAsDouble();
        double rate = encoderRate.getAsDouble();

        double battery = RobotController.getBatteryVoltage();

        double motorVolts = talon.motor.getMotorOutputVoltage();

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);

        // command motors to do things
        talon.motor.set(ControlMode.PercentOutput, autospeed);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = motorVolts;
        numberArray[4] = position;
        numberArray[5] = rate;
        telemetryEntry.setNumberArray(numberArray);

        return false;
    }

    @Override
    public boolean isActive()
    {
        return false;
    }

    @Override
    public void cancel()
    {

    }
}