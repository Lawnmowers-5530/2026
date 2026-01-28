package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private TalonFX m_turretRotation;
    private TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private DynamicMotionMagicExpoVoltage turretControl;

    private TalonFX m_turretElevation;
    private TalonFXConfiguration elevationConfig = new TalonFXConfiguration();
    private DynamicMotionMagicExpoVoltage elevationControl;

    private TalonFX m_flywheel;
    private TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private DynamicMotionMagicExpoVoltage flywheelControl;

    public Turret() {
        var slot0turretConfig = turretConfig.Slot0;
        slot0turretConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0turretConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0turretConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0turretConfig.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0turretConfig.kI = 0; // no output for integrated error
        slot0turretConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var slot0elevationConfig = elevationConfig.Slot0;
        slot0elevationConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0elevationConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0elevationConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0elevationConfig.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0elevationConfig.kI = 0; // no output for integrated error
        slot0elevationConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var slot0flywheelConfig = flywheelConfig.Slot0;
        slot0flywheelConfig.kS = 0.2; // Add 0.2 V output to overcome static friction
        slot0flywheelConfig.kV = 0.1; // A velocity target of 1 rps results in 0.1 V output
        slot0flywheelConfig.kA = 0.005; // An acceleration of 1 rps/s requires 0.005 V output
        slot0flywheelConfig.kP = 3.0; // A position error of 2.5 rotations results in 12 V output
        slot0flywheelConfig.kI = 0; // no output for integrated error
        slot0flywheelConfig.kD = 0.05; // A velocity error of 1 rps results in 0.05 V output

        this.m_turretRotation = new TalonFX(18);
        this.m_turretRotation.getConfigurator().apply(turretConfig);
        this.turretControl = new DynamicMotionMagicExpoVoltage(0, 0, 0).withEnableFOC(true).withSlot(0);
        this.m_turretRotation.setControl(turretControl);

        this.m_turretElevation = new TalonFX(19);
        this.m_turretElevation.getConfigurator().apply(elevationConfig);
        this.elevationControl = new DynamicMotionMagicExpoVoltage(0, 0, 0).withEnableFOC(true).withSlot(0);
        this.m_turretElevation.setControl(elevationControl);

        this.m_flywheel = new TalonFX(20);
        this.m_flywheel.getConfigurator().apply(flywheelConfig);
        this.flywheelControl = new DynamicMotionMagicExpoVoltage(0, 0, 0).withEnableFOC(true).withSlot(0);
        this.m_flywheel.setControl(flywheelControl);
    }

    public void setHorizontalPosition(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRadians(); // adjust by gear ratio / sensor units as needed
        this.turretControl.Position = targetPosition;
        this.m_turretRotation.setControl(this.turretControl);
    }

    public void setVerticalPosition(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRadians(); // adjust by gear ratio / sensor units as needed
        this.elevationControl.Position = targetPosition;
        this.m_turretElevation.setControl(this.elevationControl);
    }

    public void setFlywheelSpeed(double speed) {
        // convert speed to controller velocity units (rps here as an example)
        double targetVelocity = speed; // adjust by gear ratio / sensor units as needed
        this.flywheelControl.Velocity = targetVelocity;
        this.m_flywheel.setControl(this.flywheelControl.withVelocity(targetVelocity));
    }
}
