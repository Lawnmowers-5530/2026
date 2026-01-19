package frc.lib;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.system.NumericalIntegration;

public class ShotCalculatorSim {
    private final double m;
    private final double g;
    private final double kDrag;
    private final double sMagnus;

    private final double wx, wy, wz; // Spin vector

    public ShotCalculatorSim(double m, double g, double rho, double Cd, double area, double sMagnus, double wx, double wy,
            double wz) {
        this.m = m;
        this.g = g;
        this.kDrag = 0.5 * rho * Cd * area;
        this.sMagnus = sMagnus;
        this.wx = wx;
        this.wy = wy;
        this.wz = wz;
    }

    /**
     * One RK4 step for the 6-state projectile:
     * y = [x, y, z, vx, vy, vz]^T
     */
    public Matrix<N6, N1> step(double tSeconds, Matrix<N6, N1> y, double dtSeconds) {
        // f(t, y) = dy/dt
        BiFunction<Double, Matrix<N6, N1>, Matrix<N6, N1>> f = (t, state) -> {
            final double x = state.get(0, 0);
            final double yPos = state.get(1, 0);
            final double z = state.get(2, 0);
            final double vx = state.get(3, 0);
            final double vy = state.get(4, 0);
            final double vz = state.get(5, 0);

            // Speed
            final double vMag = Math.sqrt(vx * vx + vy * vy + vz * vz);

            // Quadratic drag force: Fd = -kDrag * |v| * v
            final double Fdx = -kDrag * vMag * vx;
            final double Fdy = -kDrag * vMag * vy;
            final double Fdz = -kDrag * vMag * vz;

            // Magnus force: Fm = sMagnus * (omega x v)
            // omega x v = [ wy*vz - wz*vy, wz*vx - wx*vz, wx*vy - wy*vx ]
            final double Fmx = sMagnus * (wy * vz - wz * vy);
            final double Fmy = sMagnus * (wz * vx - wx * vz);
            final double Fmz = sMagnus * (wx * vy - wy * vx);

            // Gravity force: Fg = -m g zHat
            final double Fgx = 0.0;
            final double Fgy = 0.0;
            final double Fgz = -m * g;

            // Acceleration
            final double ax = (Fdx + Fmx + Fgx) / m;
            final double ay = (Fdy + Fmy + Fgy) / m;
            final double az = (Fdz + Fmz + Fgz) / m;

            // State derivative: [xDot, yDot, zDot, vxDot, vyDot, vzDot]
            return new Matrix<>(Nat.N6(), Nat.N1(), new double[] {
                    vx, // xDot
                    vy, // yDot
                    vz, // zDot
                    ax, // vxDot
                    ay, // vyDot
                    az // vzDot
            });
        };

        // WPILib RK4 integration: y_{n+1} = rk4(f, t, y_n, dt)
        return NumericalIntegration.rk4(f, tSeconds, y, dtSeconds);
    }

    /**
     * Utility to make the initial state vector.
     */
    public static Matrix<N6, N1> makeState(double x, double y, double z,
            double vx, double vy, double vz) {
        return new Matrix<>(Nat.N6(), Nat.N1(), new double[] { x, y, z, vx, vy, vz });
    }

    public static SimResult sim(ShotCalculatorSim simCalc, double tMax, double dt, Matrix<N6, N1> initialState,
            Matrix<N3, N1> targetPosition, Matrix<N2, N1> robotVelocity) {

        double t = 0.0;
        Matrix<N6, N1> state = initialState.copy();

        state.set(3, 0, state.get(3, 0) + robotVelocity.get(0,0));
        state.set(4, 0, state.get(4, 0) + robotVelocity.get(1,0));
        

        double minDist = Double.MAX_VALUE;
        double tClosest = 0.0;
        Matrix<N3, N1> closestPos = VecBuilder.fill(0.0, 0.0, 0.0);

        for (int i = 0; i < (tMax - t) / dt; i++) {
            state = simCalc.step(t, state, dt);
            t += dt;

            double x = state.get(0, 0);
            double y = state.get(1, 0);
            double z = state.get(2, 0);
            double dist = Math.sqrt(Math.pow(x - targetPosition.get(0, 0), 2) +
            Math.pow(y - targetPosition.get(1, 0), 2) +
            Math.pow(z - targetPosition.get(2, 0), 2));

            if (dist < minDist) {
                minDist = dist;
                tClosest = t;
                closestPos = VecBuilder.fill(x, y, z);
            }

            // System.out.printf("t=%.3f s: x=%.2f m, y=%.2f m, z=%.2f m%n", t, x, y, z);

            if (z <= 0.0) {
                break; // Hit the ground
            }
        }
        return new SimResult(Vec3.fromVector(closestPos), tClosest, minDist);
    }
}

class Vec3 {
    public double x, y, z;

    public Vec3(double x, double y, double z) {
        this.x = x; this.y = y; this.z = z;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public Vec3 add(Vec3 o) {
        return new Vec3(x + o.x, y + o.y, z + o.z);
    }

    public Vec3 sub(Vec3 o) {
        return new Vec3(x - o.x, y - o.y, z - o.z);
    }

    public Vec3 scale(double s) {
        return new Vec3(x * s, y * s, z * s);
    }

    public double dot(Vec3 o) {
        return x * o.x + y * o.y + z * o.z;
    }

    public double mag() {
        return Math.sqrt(dot(this));
    }

    public Vec3 normalize() {
        double m = mag();
        if (m == 0) return this;
        return scale(1.0 / m);
    }

    public static Vec3 clampMagnitude(Vec3 v, double maxMag) {
        double m = v.mag();
        if (m <= maxMag) return v;
        return v.scale(maxMag / m);
    }

    public static Vec3 fromVector(Matrix<N3, N1> vec) {
        return new Vec3(vec.get(0, 0), vec.get(1, 0), vec.get(2, 0));
    }
}


class SimResult {
    public final Vec3 closestPos;
    public final double tClosest;
    public final double err;

    public SimResult(Vec3 closestPos, double tClosest, double err) {
        this.closestPos = closestPos;
        this.tClosest = tClosest;
        this.err = err;
    }
}

