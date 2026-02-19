package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public class SwerveConstants {
        public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
        public static final double MaxAcceleration = 3.0; // max acceleration in m/s^2
        public static final double MaxAngularVelocity = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 1 rotation per second
                                                                                      // max angular velocity
        public static final double MaxAngularAcceleration = 1.0; // max angular acceleration in rad/s^2


    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
}
