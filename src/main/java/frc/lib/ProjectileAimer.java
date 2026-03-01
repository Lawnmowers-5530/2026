package frc.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import frc.lib.ShotCalculatorSim;
import frc.robot.subsystems.Turret;

public class ProjectileAimer {
    private final ShotCalculatorSim shotCalculator;
    private final double m;
    private final double g;
    private final double rho;
    private final double Cd;
    private final double area;
    private final double sMagnus;

    private final double wx;
    private final double wy;
    private final double wz;

    private final double tMax;
    private final double dt;

    private final double V = 8; //Predetermined exit velocity from shooter

    private final double maxIter;
    private final Matrix<N6, N1> guessState;
    private final Matrix<N3, N1> targetPosition;
    private final Matrix<N6, N1> initialState;
    private final Matrix<N2, N1> robotVelocity;

    public ProjectileAimer(Matrix<N6, N1> initialState, Matrix<N3, N1> targetPosition, Matrix<N2, N1> robotVelocity) {
        this.m = 0.2154; // mass of projectile in kg (e.g., baseball)
        this.g = 9.81; // gravity in m/s^2
        this.rho = 1.225; // air density at sea level in kg/m^3
        this.Cd = 0;//0.3; // drag coefficient (typical for a sphere)
        this.area = Math.PI * Math.pow(0.150622, 2);
        this.sMagnus = 1.0e-4; // Magnus effect coefficient (tunable)

        this.wx = 0.0; // spin around x-axis in rad/s
        this.wy = 0; // spin around y-axis in rad/s
        this.wz = 0; // spin around z-axis in rad/s

        this.tMax = 1.75; // maximum flight time in seconds
        this.dt = 0.01; // time step for simulation in seconds

        this.targetPosition = targetPosition;
        this.initialState = initialState;
        this.robotVelocity = robotVelocity;

        this.maxIter = 50; // maximum iterations for aiming calculations
        this.shotCalculator = new ShotCalculatorSim(m, g, rho, Cd, area, sMagnus, wx, wy, wz);
        this.guessState = this.initialState;
    }

    public void printResults() {
        System.out.println("x velo:" + this.guessState.get(3, 0));
        System.out.println("y velo:" + this.guessState.get(4, 0));
        System.out.println("z velo:" + this.guessState.get(5, 0));
    }

    public Matrix<N6, N1> loop() {
        for (int i = 0; i < maxIter; i++) {
            SimResult result = ShotCalculatorSim.sim(this.shotCalculator, this.tMax, this.dt, this.guessState,
                    this.targetPosition, this.robotVelocity);

            Vec3 v0New = updateLaunchVelocity(new Vec3(this.guessState.get(3, 0), this.guessState.get(4, 0), this.guessState.get(5, 0)),
             Vec3.fromVector(this.targetPosition), result, V, 0.3, Math.toRadians(2));

            this.guessState.set(3, 0, v0New.x);
            this.guessState.set(4, 0, v0New.y);
            this.guessState.set(5, 0, v0New.z);


        }
        return this.guessState;
    }

    public static Vec3 updateLaunchVelocity(
            Vec3 v0, // current initial velocity (|v0| fixed)
            Vec3 targetPos,
            SimResult sim,
            double speed,
            double gain,
            double maxAngleStepRad) {
        // 1. Error at closest approach
        Vec3 e = sim.closestPos.sub(targetPos);

        // Safety: avoid divide-by-zero
        if (sim.tClosest < 1e-6) {
            return v0;
        }

        // 2. Desired (unconstrained) velocity correction
        Vec3 dvDesired = e.scale(-1.0 / sim.tClosest);

        // 3. Project onto tangent plane (remove parallel component)
        Vec3 vHat = v0.normalize();
        double parallel = dvDesired.dot(vHat);
        Vec3 dvTangent = dvDesired.sub(vHat.scale(parallel));

        // 4. Convert to direction update
        Vec3 deltaDir = dvTangent.scale(1.0 / speed);

        // Apply gain and clamp angular step
        deltaDir = deltaDir.scale(gain);
        deltaDir = Vec3.clampMagnitude(deltaDir, maxAngleStepRad);

        // 5. Rotate direction and reconstruct velocity
        Vec3 newDir = vHat.add(deltaDir).normalize();
        return newDir.scale(speed);
    }

    //public static void main(String[] args) {
    //    ProjectileAimer aimer = new ProjectileAimer(
    //        VecBuilder.fill(0, 0, 0, 3, 3, 20),
    //        VecBuilder.fill(2, 2, 2),
    //        VecBuilder.fill(0, 0)
    //    );
    //    long t0 = System.nanoTime();
    //    System.out.println(aimer.loop());
    //    long t1 = System.nanoTime();
//
    //    System.out.printf("Time taken: %.3f ms%n", (t1 - t0) / 1e6);
//
    //    Matrix<N1, N1> test = VecBuilder.fill(5);
    //    Matrix<N1, N1> test2 = test.copy();
    //    test2.set(0, 0, 10);
    //    System.out.println(test);
    //}
    public static double findv0(double r, Rotation2d angle, double h) {
        return Math.sqrt(9.81 * Math.pow(r, 2))/(2 * Math.pow(Math.cos(angle.getRadians()), 2)*(r*Math.tan(angle.getRadians())+h));
    }

public static Turret.TurretState optimizeTurretState(
    Translation3d target, 
    Vector<N2> robotTranslation, 
    Vector<N2> turretVelocity,
    double maxVelocity,
    double minPitchDegrees, // Added
    double maxPitchDegrees
) {
    double bestDzDt = 0.0; 
    double low = -15.0; // Steepest search limit
    double high = 0.0;  // Flattest search limit
    
    for (int i = 0; i < 15; i++) {
        double mid = (low + high) / 2.0;
        Turret.TurretState state = parabolicTurretState(target, robotTranslation, turretVelocity, mid);
        
        double currentPitch = state.pitch.getDegrees();

        // LOGIC CHECK:
        // 1. If pitch is too LOW, we need a steeper shot (move toward 'low')
        // 2. If pitch is too HIGH or velocity is too FAST, we need a flatter shot (move toward 'high')
        
        if (currentPitch < minPitchDegrees) {
            // Shot is too flat for the hardware; try to make it steeper
            high = mid; 
        } else if (currentPitch > maxPitchDegrees || state.flywheelSpeed > maxVelocity) {
            // Shot is too steep or too fast; try to make it flatter
            low = mid;
        } else {
            // This dz/dt works! Save it and try to see if we can get even steeper
            bestDzDt = mid;
            high = mid; 
        }
    }
    
    return parabolicTurretState(target, robotTranslation, turretVelocity, bestDzDt);
}

public static Turret.TurretState parabolicTurretState(Translation3d target, Vector<N2> robotTranslation, Vector<N2> turretVelocity, double dzdt) {
    // 1. Constants
    final double g = 9.80665; // Gravity m/s^2
    // Constraint: Vertical velocity at target (dz/dt). 
    // Usually negative for a "descending" hit (swish).
    final double dz_dt_constraint = dzdt; 



    // 2. Calculate Displacements
    Translation3d relativeTranslation = target.minus(new Translation3d(robotTranslation.get(0), robotTranslation.get(1), 0));
    double dx = relativeTranslation.getX();
    double dy = relativeTranslation.getY();
    double dz = relativeTranslation.getZ();
    double horizontalDist = Math.hypot(dx, dy);

    // 3. Solve for Time of Flight (t)
    // Formula derived from: dz = (v_zt + g*t)*t - 0.5*g*t^2  =>  0.5*g*t^2 + v_zt*t - dz = 0
    // Quadratic formula: t = [-v_zt + sqrt(v_zt^2 - 4(0.5g)(-dz))] / (2 * 0.5g)
    double discriminant = Math.pow(dz_dt_constraint, 2) + 2 * g * dz;
    
    if (discriminant < 0) {
        // Target is physically unreachable with the given dz/dt constraint
        return new Turret.TurretState(new Rotation2d(), new Rotation2d(), 0); 
    }

    double t = (-dz_dt_constraint + Math.sqrt(discriminant)) / g;

    // 4. Calculate Required World Velocity Components
    double vx_world = dx / t;
    double vy_world = dy / t;
    double vz_initial = dz_dt_constraint + g * t;

    // 5. Compensate for Robot Velocity
    // Robot velocity is Vector<N2> (x, y). Subtract it from world velocity 
    // to get the velocity the shooter needs to generate.
    double vx_shooter = vx_world - turretVelocity.get(0);
    double vy_shooter = vy_world - turretVelocity.get(1);
    double vz_shooter = vz_initial; // Assuming robot vertical velocity is 0

    // 6. Convert to Spherical Coordinates (Yaw, Pitch, Magnitude)
    double horizontalVelocityShooter = Math.hypot(vx_shooter, vy_shooter);
    
    Rotation2d yaw = new Rotation2d(vx_shooter, vy_shooter);
    Rotation2d pitch = new Rotation2d(horizontalVelocityShooter, vz_shooter);
    double exitVelocity = Math.sqrt(Math.pow(horizontalVelocityShooter, 2) + Math.pow(vz_shooter, 2));

    return new Turret.TurretState(yaw, pitch, exitVelocity);
}

public static void main(String[] args) {
    Translation3d target = new Translation3d(2, 2, 2);
    Vector<N2> robotPose = VecBuilder.fill(2, 0);
    Vector<N2> turretVelocity = VecBuilder.fill(1, 0);
    long t0 = System.nanoTime();
    Turret.TurretState state = optimizeTurretState(target, robotPose, turretVelocity, 10.0, 52, 71.0);
    long t1 = System.nanoTime();
    System.out.printf("Time taken: %.3f ms%n", (t1 - t0) / 1e6);
    System.out.println(state.toString());
}
}

//TODO remove matrices and replace args for construction with vectors that are wayyyy easier to make