package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.lib.ProjectileAimer;

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
    public static final double motorToPitchRot = 17 * 0.75;
    public static final double motorToFlywheelRot = 1;
    public static final double sysIdRampRate = 1.0; // Volts per second
    public static final double sysIdDynamicStepVoltage = 0.0; // Volts
    public static final double sysIdTimeout = 10.0; // Seconds
    public static final Translation2d blueTargetPose = new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(40));//new Translation2d(4.619, 4.027);
    public static final Rotation2d dragChainZeroAngle = Rotation2d.fromDegrees(315);
    public static final Translation2d distFromCenter = new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(-6));
    public static final double launcherHeight = 0.3;
    public static InterpolatingTreeMap<Double, Double> VelocityToRPS =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
     static {
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(27), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 25.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(89), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 40.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(128), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 50.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(156), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 55.0);


        //add more here
    }
}