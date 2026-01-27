package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private TalonFX m_turretRotation;
    private TalonFXConfiguration turretConfig = new TalonFXConfiguration();


    public Turret() {
        this.m_turretRotation = new TalonFX(18);
        this.m_turretRotation.getConfigurator().apply(turretConfig)
    }
}
