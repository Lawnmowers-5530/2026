package frc.lib;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.lang.Math;

public class ShotCalculator {
    // Constant for gravity (m/s^2)
    private static final double G = 9.80665;
    // Fixed pitch in radians
    private static final double PITCH_RADS = Math.toRadians(72.0);

    /**
     * Calculates required initial velocity (m/s) to hit a target.
     * @param current The robot's current Translation3d position.
     * @param target The target's Translation3d position.
     * @return Required launch velocity in meters per second.
     */
    public static double calculateVelocity(Translation3d current, Translation3d target) {
        // 1. Calculate horizontal distance (d) in the XY plane
        double dx = target.getX() - current.getX();
        double dy = target.getY() - current.getY();
        double d = Math.sqrt(dx * dx + dy * dy);

        // 2. Calculate vertical displacement (h)
        double h = target.getZ() - current.getZ();

        // 3. Pre-calculate trig values for efficiency
        double cosTheta = Math.cos(PITCH_RADS);
        double tanTheta = Math.tan(PITCH_RADS);

        // 4. Apply the projectile motion formula
        // Denominator: 2 * cos^2(theta) * (d * tan(theta) - h)
        double denominator = 2 * Math.pow(cosTheta, 2) * (d * tanTheta - h);

        // Guard against imaginary results (target out of reach for this angle)
        if (denominator <= 0) {
            return 0.0; 
        }

        return Math.sqrt((G * d * d) / denominator);
    }

    public static void main(String[] args) {
        System.out.println(calculateVelocity(new Translation3d(), new Translation3d(4, 4, Units.feetToMeters(6))));
    }

    
}
