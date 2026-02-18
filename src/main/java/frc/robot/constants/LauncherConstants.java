package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import lombok.experimental.UtilityClass;

@UtilityClass
public class LauncherConstants { //TODO: fill in constants
    public final CANBus canBus = new CANBus("canivore", "logs/launcherFlywheel");
    public final int canId = 23;
    public final double kV = 0.12379;
    public final double kA = 0.011841;
    public final double kS = 0.34;
    public final double kP = 0.19033;
    public final double kI = 0.0;
    public final double kD = 0.00;
    public final double motorToYawRot = 8;
    public final double motorToPitchRot = 17;
    public final double motorToFlywheelRot = 12/18;
    public final double sysIdRampRate = 1.0; // Volts per second
    public final double sysIdDynamicStepVoltage = 0.0; // Volts
    public final double sysIdTimeout = 10.0; // Seconds
}
