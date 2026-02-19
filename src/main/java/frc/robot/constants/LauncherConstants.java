package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class LauncherConstants { //TODO: fill in constants
    public static final CANBus canBus = new CANBus("canivore", "logs/launcherFlywheel");
    public static final int canId = 23;
    public static final double kV = 0.12379;
    public static final double kA = 0.011841;
    public static final double kS = 0.34;
    public static final double kP = 0.19033;
    public static final double kI = 0.0;
    public static final double kD = 0.00;
    public static final double motorToYawRot = 8;
    public static final double motorToPitchRot = 17;
    public static final double motorToFlywheelRot = 1;
    public static final double sysIdRampRate = 1.0; // Volts per second
    public static final double sysIdDynamicStepVoltage = 0.0; // Volts
    public static final double sysIdTimeout = 10.0; // Seconds
}