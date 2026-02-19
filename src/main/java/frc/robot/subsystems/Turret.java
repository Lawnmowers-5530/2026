package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;

public class Turret extends SubsystemBase {
    private TalonFX m_yaw;
    private TalonFXConfiguration yawConfig = new TalonFXConfiguration();
    private MotionMagicVoltage yawControl;

    private TalonFX m_pitch;
    private TalonFXConfiguration pitchConfig = new TalonFXConfiguration();
    private MotionMagicVoltage pitchControl;

    private TalonFX m_flywheel;
    private TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private VelocityTorqueCurrentFOC flywheelControl;

    public Turret() {
        var slot0yawConfig = yawConfig.Slot0;
        slot0yawConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0yawConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0yawConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0yawConfig.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0yawConfig.kI = 0; // no output for integrated error
        slot0yawConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigYaw = yawConfig.MotionMagic;
        motionMagicConfigYaw.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigYaw.MotionMagicAcceleration = 300; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigYaw.MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        var slot0pitchConfig = pitchConfig.Slot0;
        slot0pitchConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0pitchConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0pitchConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0pitchConfig.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0pitchConfig.kI = 0; // no output for integrated error
        slot0pitchConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigpitch = pitchConfig.MotionMagic;
        motionMagicConfigpitch.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigpitch.MotionMagicAcceleration = 300; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigpitch.MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        var slot0flywheelConfig = flywheelConfig.Slot0;
        slot0flywheelConfig.kS = 0.2; // Add 0.2 V output to overcome static friction
        slot0flywheelConfig.kV = 0.1; // A velocity target of 1 rps results in 0.1 V output
        slot0flywheelConfig.kA = 0.005; // An acceleration of 1 rps/s requires 0.005 V output
        slot0flywheelConfig.kP = 3.0; // A position error of 2.5 rotations results in 12 V output
        slot0flywheelConfig.kI = 0; // no output for integrated error
        slot0flywheelConfig.kD = 0.05; // A velocity error of 1 rps results in 0.05 V output

        var torqueCurrentConfigFlywheel = flywheelConfig.TorqueCurrent;
        torqueCurrentConfigFlywheel.PeakForwardTorqueCurrent = 500;
        torqueCurrentConfigFlywheel.PeakReverseTorqueCurrent = -500;
        torqueCurrentConfigFlywheel.TorqueNeutralDeadband = 0;

        this.m_yaw = new TalonFX(21, "canivore");
        this.m_yaw.getConfigurator().apply(yawConfig);
        this.yawControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        this.m_yaw.setControl(yawControl);

        pitchConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_pitch = new TalonFX(22, "canivore");
        this.m_pitch.getConfigurator().apply(pitchConfig);
        this.pitchControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        this.m_pitch.setControl(pitchControl);

        pitchConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_flywheel = new TalonFX(23, "canivore");
        this.m_flywheel.getConfigurator().apply(flywheelConfig);
        this.flywheelControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
        this.m_flywheel.setControl(flywheelControl);
    }

    public void setHorizontalPosition(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRotations() * LauncherConstants.motorToYawRot;
        double spModular = targetPosition % (LauncherConstants.motorToYawRot); // wrap setpoint to [0, 2pi)
        if (spModular < 0) {
            spModular = LauncherConstants.motorToYawRot + spModular;
        }
        this.m_yaw.setControl(this.yawControl.withPosition(spModular));
        System.out.println("spModular: " + spModular);

        System.out.println("controller pos set: " + this.yawControl.Position);

        System.out.println("output speed: " + this.m_yaw.get());

        SmartDashboard.putString("Encoder Pos", this.m_yaw.getPosition().toString());
    }

    public void setVerticalPosition(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRotations() * LauncherConstants.motorToPitchRot;
        this.pitchControl.Position = targetPosition;
        this.m_pitch.setControl(this.pitchControl);
    }

    public void setFlywheelSpeed(double speed) {
        // convert speed to controller velocity units (rps here as an example)
        double targetVelocity = speed * LauncherConstants.motorToFlywheelRot; // adjust by gear ratio / sensor units as needed
        //this.m_flywheel.setControl(this.flywheelControl.withVelocity(targetVelocity));
        this.m_flywheel.setVoltage(speed);
    }

    public void zeroYaw() {
        this.m_yaw.setPosition(0);
    }

    public void zeroPitch() {
        this.m_pitch.setPosition(0);
    }
}
