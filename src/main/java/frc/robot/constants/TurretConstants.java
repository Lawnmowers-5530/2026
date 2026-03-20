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

    public final Rotation2d turretOffset = Rotation2d.fromRotations(-1.09277).times(1.0 / 8.0);
    public final double turretYawMin = -79.65 + 10;
    public final double turretYawMax = 115.2 - 10;
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
    
        yaw_kS = 0.6527,
        yaw_kV = 0.1321,
        yaw_kA = 0.01178,
        yaw_kP = 18,
        yaw_kI = 0.0,
        yaw_kD = 0,
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
        flywheel_kS = 0.34,
        flywheel_kV = 0.12379,
        flywheel_kA = 0.011841,
        flywheel_kP = 0.19033,
        flywheel_kI = 0.0,
        flywheel_kD = 0.0;

    public final TurretState state1 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);
    public final TurretState state2 = new TurretState(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(70), 7);

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
       
        addNumberToKeys(78.73*0.0254, 52, 0, 72);

        addNumberToKeys(98.2*0.0254, 60.0,0, 70);
       
        addNumberToKeys(121.73*0.0254, 62,0, 68.5);

        addNumberToKeys(143.73*0.0254, 67, 0, 68);

        addNumberToKeys(158.73*0.0254, 69.5, 0, 66);

        addNumberToKeys(171.23*0.0254, 72, 0, 65);

        addNumberToKeys(183.23*0.0254, 74, 0, 64);

        addNumberToKeys(219.23*0.0254, 82, 0, 62);

        addNumberToKeys(10, 85, 0, 59);

        addNumberToKeys(15, 88, 0, 53);

    }

    static void addNumberToKeys(double distance, double rps, double tof, double rotationDegs) {
        distToSpinrate.put(distance,rps);
        launchHoodAngleMap.put(distance, Rotation2d.fromDegrees(rotationDegs));
        distToTOF.put(distance, tof);
    }

    

}