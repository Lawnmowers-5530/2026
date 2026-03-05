package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class Test {
        public static InterpolatingTreeMap<Double, Double> VelocityToRPS =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
     static {
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(27), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 25.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(89), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 40.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(128), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 50.0);
        VelocityToRPS.put(ProjectileAimer.findv0(Units.inchesToMeters(156), Rotation2d.fromDegrees(76), Units.inchesToMeters(15.5)), 55.0);


        //add more here
    }

    public static void main(String[] args) {
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(59.75), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(82), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(115), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(154.5), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(144+96), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
        System.out.println(ProjectileAimer.findv0(Units.inchesToMeters(144+124), Rotation2d.fromDegrees(65), Units.inchesToMeters(15.5)));
    }
}
