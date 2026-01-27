package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private TalonFX m_turretRotation;
    private TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private PositionTorqueCurrentFOC turretControl;

    public Turret() {
        this.m_turretRotation = new TalonFX(18);
        this.m_turretRotation.getConfigurator().apply(turretConfig);
        this.turretControl = new PositionTorqueCurrentFOC(0).withFeedForward(Current.ofBaseUnits(1, Units.Amps));
        this.m_turretRotation.setControl(turretControl);
    }

    public void setHorizontalPosition(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        double targetPosition = pos.getRadians(); // adjust by gear ratio / sensor units as needed
        this.turretControl.Position = targetPosition;
        this.m_turretRotation.setControl(this.turretControl);
    }
}
