package frc.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import frc.lib.ShotCalculatorSim;

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

    public static void main(String[] args) {
        ProjectileAimer aimer = new ProjectileAimer(
            VecBuilder.fill(0, 0, 0, 3, 3, 20),
            VecBuilder.fill(2, 2, 2),
            VecBuilder.fill(0, 0)
        );
        long t0 = System.nanoTime();
        System.out.println(aimer.loop());
        long t1 = System.nanoTime();

        System.out.printf("Time taken: %.3f ms%n", (t1 - t0) / 1e6);

        Matrix<N1, N1> test = VecBuilder.fill(5);
        Matrix<N1, N1> test2 = test.copy();
        test2.set(0, 0, 10);
        System.out.println(test);
    }
}

//TODO remove matrices and replace args for construction with vectors that are wayyyy easier to make