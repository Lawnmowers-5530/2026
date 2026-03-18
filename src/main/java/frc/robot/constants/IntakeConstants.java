package frc.robot.constants;

import lombok.experimental.UtilityClass;

@UtilityClass
public class IntakeConstants {
    //TODO find the actual values for these
    public final int
        pivotMotorPort = 24,
        runMotorPort = 25;

    public final String dashboardPath = "Intake";

    // Grouped PID / motion constants
    public double
        pivotKS = 0.5,
        pivotKV = 0,
        pivotKA = 0,
        pivotKP = 12,
        pivotKI = 0,
        pivotKD = 0,
        pivotGravityArmPositionOffset = 0.1,
        pivotKG = 0,
        pivotMotionMagicCruiseVelocity = 20,
        pivotMotionMagicAcceleration = 0.5,
        pivotMotionMagicJerk = 0.5,
        lowerJigglePos = 14,
        upperJigglePos = 7;


    public double
        runMotorAmps = 15,
        //pivotHoldDownAmps = 10,
        pivotTorqueDownwardAmps = 15.0;

    public double
        // New constants to remove magic numbers from Intake.java
        // Debounce time for stall detection (seconds)
        stallDebounceSeconds = 0.05,
        // Stall detection thresholds
        stallCurrentAmpsThreshold = 100, // amps
        stallVelocityRpsThreshold = 5;

    public double
        // Voltages used for moving/holding the pivot and running the intake
        tuckVoltage = -3.0,
        tuckHoldVoltage = -0.4,
        extendVoltage = 2.0,
        runMotorVoltage = 11.5;

    public double
        pivotPositionTolerance = 0.001,
        extendedEncoderPosition = 17.21582,
        tuckedEncoderPosition = 0;
}
