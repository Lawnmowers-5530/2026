package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private TorqueCurrentFOC flywheelControl;

    public Turret() {
        CommandScheduler.getInstance().registerSubsystem(this);

        var slot0yawConfig = yawConfig.Slot0;
        slot0yawConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0yawConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0yawConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0yawConfig.kP = 0.25; // A position error of 2.5 rotations results in 12 V output
        slot0yawConfig.kI = 0; // no output for integrated error
        slot0yawConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigYaw = yawConfig.MotionMagic;
        motionMagicConfigYaw.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigYaw.MotionMagicAcceleration = 20; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigYaw.MotionMagicJerk = 50; // Target jerk of 1600 rps/s/s (0.1 seconds)

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
        slot0flywheelConfig.kP = 0; // A position error of 2.5 rotations results in 12 V output
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

        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_flywheel = new TalonFX(23, "canivore");
        this.m_flywheel.getConfigurator().apply(flywheelConfig);
        this.flywheelControl = new TorqueCurrentFOC(0);
        this.m_flywheel.setControl(flywheelControl);
    }

    public void setYaw(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRadians();
        Rotation2d spModular = Rotation2d.fromRadians(MathUtil.angleModulus(targetPosition) * LauncherConstants.motorToYawRot);

        this.yawControl.Position = spModular.getRotations();
        this.m_yaw.setControl(this.yawControl);
        SmartDashboard.putNumber("spModular", spModular.getRotations());

        SmartDashboard.putString("Encoder Pos", this.m_yaw.getPosition().toString());
    }

    public void setPitch(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRotations() * LauncherConstants.motorToPitchRot;
        this.pitchControl.Position = targetPosition;
        this.m_pitch.setControl(this.pitchControl);
    }

    public void setFlywheelSpeed(double speed) {
        // convert speed to controller velocity units (rps here as an example)
        double targetVelocity = speed * LauncherConstants.motorToFlywheelRot; // adjust by gear ratio / sensor units as needed
        
        //this.m_flywheel.setControl(this.flywheelControl.withVelocity(targetVelocity));
        //this.m_flywheel.setControl(velocityRequest);
        this.flywheelControl.Output = targetVelocity;
        this.m_flywheel.setControl(this.flywheelControl);
    }

    public void zeroYaw() {
        this.m_yaw.setPosition(0);
    }

    public void zeroPitch() {
        this.m_pitch.setPosition(0);
    }

    public Command setFlywheelSpeedCommand(double speed) {
        return new InstantCommand(() -> {this.setFlywheelSpeed(speed);}, this);
    }

    public Command setPitchCommand(Rotation2d angle) {
        return new InstantCommand(() -> {this.setPitch(angle);}, this);
    }

    public void periodic() {
        
        SmartDashboard.putString("pitch request", this.pitchControl.getControlInfo().toString());
    }
}
