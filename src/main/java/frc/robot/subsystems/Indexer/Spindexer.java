package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    TalonFX spindexer = new TalonFX(26, "canivore");
    TalonFX kicker = new TalonFX(27, "canivore");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    DutyCycleOut spindexerControl = new DutyCycleOut(0.5);
    DutyCycleOut kickerControl = new DutyCycleOut(1);



    public Spindexer() {
        this.spindexer.getConfigurator().apply(spindexerConfig);
        this.kicker.getConfigurator().apply(kickerConfig);
        spindexerControl.EnableFOC = true;
        kickerControl.EnableFOC = true;
    };

    public void spinKick() {
        spindexerControl.Output = 0.5;
        this.spindexer.setControl(spindexerControl);
        this.kicker.setControl(kickerControl);
    }

    public void stopSpinKick() {
        this.spindexer.stopMotor();
        this.kicker.stopMotor();
    }

    public Command spinKickCommand() {
        return Commands.runOnce(() -> {this.spinKick();}, this);
    }

    public Command stopSpinKickCommand() {
        return Commands.runOnce(() -> {this.stopSpinKick();});
    }
}
