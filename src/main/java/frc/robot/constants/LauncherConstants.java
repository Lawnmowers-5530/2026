package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.lib.ProjectileAimer;
import frc.robot.subsystems.Turret.TurretState;

public class LauncherConstants { // TODO: fill in constants
    public static final CANBus canBus = new CANBus("canivore", "logs/launcherFlywheel");
    public static final int canId = 23;
    public static final double kV = 0.12379;
    public static final double kA = 0.011841;
    public static final double kS = 0.34;
    public static final double kP = 0.19033;
    public static final double kI = 0.0;
    public static final double kD = 0.00;
    public static final double motorToYawRot = 8;
    public static final double motorRotToPitchDeg = 1.4 / 20.0;// 17 * 0.75;
    public static final Rotation2d turretOffset = Rotation2d.fromRotations(0.647).times(1.0 / 8.0);
    public static final Rotation2d pitchZeroAngle = Rotation2d.fromDegrees(72);
    public static final double motorToFlywheelRot = 1;
    public static final double sysIdRampRate = 1.0; // Volts per second
    public static final double sysIdDynamicStepVoltage = 0.0; // Volts
    public static final double sysIdTimeout = 10.0; // Seconds
    public static final Translation3d blueTargetPose = new Translation3d(Units.inchesToMeters(45), 0, 0);//new Translation3d(4.619, 4.027, Units.feetToMeters(6));
    public static final Translation3d redTargetPose = new Translation3d(Units.inchesToMeters(40),
            Units.inchesToMeters(40), Units.inchesToMeters(0)); // TODO change
    public static final Translation2d distFromCenter = new Translation2d(Units.inchesToMeters(-4.25),
            Units.inchesToMeters(5.75));
    public static final double launcherHeight = 0.3;
    public static final double feedTime = 0.0; // TODO try at 0 if not working

    public static final TurretState state1 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);
    public static final TurretState state2 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);

    public static InterpolatingTreeMap<Double, Double> VelocityToRPS = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        VelocityToRPS.put(0.0, 0.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(27), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 25.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(89), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 40.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(128), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 50.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(156), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 55.0);

        // add more here
    }

    public static InterpolatingTreeMap<Double, Double> distToSpinrate = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        distToSpinrate.put(1.524, 42.0);
        distToSpinrate.put(2.286, 46.0);
        distToSpinrate.put(3.048, 50.0);
        distToSpinrate.put(3.81, 54.0);
        distToSpinrate.put(4.972, 59.0);
        distToSpinrate.put(5.334, 61.5);
    }

    public static InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);

    static {
        launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(90-19.0));
        launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(90-19.0));
        launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(90-24.0));
        launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(90-27.0));
        launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(90-29.0));
        launchHoodAngleMap.put(5.6, Rotation2d.fromDegrees(90-29.0));
    }

        public static InterpolatingTreeMap<Double, Double> distToTOF = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        distToTOF.put(1.524, 0.9025);
        distToTOF.put(2.286, 0.98);
        distToTOF.put(3.81, 1.235);
        distToTOF.put(4.572, 1.353);
        distToTOF.put(5.334, 1.413);
    }
}