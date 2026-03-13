package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.lib.ProjectileAimer;
import frc.robot.subsystems.Turret.TurretState;
import lombok.experimental.UtilityClass;

@UtilityClass
public class TurretConstants { // TODO: fill in constants
    public final CANBus canBus = RobotConstants.canivoreBus;
    public int canId = 23;

    // Dashboard path for tuning
    public static final String dashboardPath = "Turret";

    // Generic feedforward / PID (kept mutable for runtime tuning)
    public double
        kV = 0.12379,
        kA = 0.011841,
        kS = 0.34,
        kP = 0.19033,
        kI = 0.0,
        kD = 0.00;

    // Gear ratios / conversions
    public double
        motorToYawRot = 8,
        motorRotToPitchDeg = 1.4 / 20.0; // 17 * 0.75;

    public final Rotation2d turretOffset = Rotation2d.fromRotations(0.647).times(1.0 / 8.0);
    public final Rotation2d pitchZeroAngle = Rotation2d.fromDegrees(72);
    public double motorToFlywheelRot = 1;

    // SysId / safety
    public double
        sysIdRampRate = 1.0, // Volts per second
        sysIdDynamicStepVoltage = 0.0, // Volts
        sysIdTimeout = 10.0; // Seconds

    public final Translation3d blueTargetPose = new Translation3d(4.619, 4.027, 0);
    public final Translation3d bluePassingPose = new Translation3d(1.8, 4, 0);
    public final Translation3d redTargetPose = new Translation3d(Units.inchesToMeters(40),
            Units.inchesToMeters(40), Units.inchesToMeters(0)); // TODO change
    public final Translation2d distFromCenter = new Translation2d(Units.inchesToMeters(-4.25),
            Units.inchesToMeters(5.75));
    public final double launcherHeight = 0.3;
    public final double feedTime = 0.0; // TODO try at 0 if not working

    // Separate PID/motion constants for each axis and flywheel (mutable for tuning)
    // Yaw
    public double
        yaw_kS = 0.4,
        yaw_kV = 0.12,
        yaw_kA = 0.01,
        yaw_kP = 4.8,
        yaw_kI = 0.0,
        yaw_kD = 0.1,
        yawMotionMagicCruiseVelocity = 16,
        yawMotionMagicAcceleration = 40,
        yawMotionMagicJerk = 2000;

    // Pitch
    public double
        pitch_kS = 0.25,
        pitch_kV = 0.12,
        pitch_kA = 0.01,
        pitch_kP = 7.0,
        pitch_kI = 0.0,
        pitch_kD = 0.0,
        pitchMotionMagicCruiseVelocity = 16,
        pitchMotionMagicAcceleration = 300,
        pitchMotionMagicJerk = 4000;

    // Flywheel
    public double
        flywheel_kS = 0.0,
        flywheel_kV = 0.0,
        flywheel_kA = 0.0,
        flywheel_kP = 0.5,
        flywheel_kI = 0.0,
        flywheel_kD = 0.0;

    public final TurretState state1 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);
    public final TurretState state2 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);

    public final InterpolatingTreeMap<Double, Double> velocityToRPS = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    public final InterpolatingTreeMap<Double, Double> distToSpinrate = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    public final InterpolatingDoubleTreeMap distToSpinratePassing = new InterpolatingDoubleTreeMap();
    public final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMapPassing = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public final InterpolatingTreeMap<Double, Double> distToTOF = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        velocityToRPS.put(0.0, 0.0);
        velocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(27), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 25.0);
        velocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(89), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 40.0);
        velocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(128), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 50.0);
        velocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(156), Rotation2d.fromDegrees(76),
                Units.inchesToMeters(15.5)), 55.0);
    }

    static {
        distToSpinrate.put(1.524, 42.0);
        distToSpinrate.put(2.286, 46.0);
        distToSpinrate.put(3.048, 50.0);
        distToSpinrate.put(3.81, 54.0);
        distToSpinrate.put(4.972, 59.0);
        distToSpinrate.put(5.334, 61.5);
    }

    static {
        launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(90 - 19.0));
        launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(90 - 19.0));
        launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(90 - 24.0));
        launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(90 - 27.0));
        launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(90 - 29.0));
        launchHoodAngleMap.put(5.6, Rotation2d.fromDegrees(90 - 29.0));
    }

    static {
        distToTOF.put(1.524, 0.9025);
        distToTOF.put(2.286, 0.98);
        distToTOF.put(3.81, 1.235);
        distToTOF.put(4.572, 1.353);
        distToTOF.put(5.334, 1.413);
    }

    static {
        distToSpinratePassing.put(1.524, 42.0);
        distToSpinratePassing.put(2.286, 46.0);
        distToSpinratePassing.put(3.048, 50.0);
        distToSpinratePassing.put(3.81, 54.0);
        distToSpinratePassing.put(4.972, 59.0);
        distToSpinratePassing.put(5.334, 61.5);
        distToSpinratePassing.put(10.0, 90.0);
    }
    static {
        //this is some bullshit I made up but hopefully it works
        launchHoodAngleMapPassing.put(1.34, Rotation2d.fromDegrees(90 - 19.0));
        launchHoodAngleMapPassing.put(1.78, Rotation2d.fromDegrees(90 - 19.0));
        launchHoodAngleMapPassing.put(2.17, Rotation2d.fromDegrees(90 - 24.0));
        launchHoodAngleMapPassing.put(2.81, Rotation2d.fromDegrees(90 - 27.0));
        launchHoodAngleMapPassing.put(3.82, Rotation2d.fromDegrees(90 - 29.0));
        launchHoodAngleMapPassing.put(5.6, Rotation2d.fromDegrees(90 - 29.0));
        launchHoodAngleMapPassing.put(10.0, Rotation2d.fromDegrees(90 - 35));
    }
}