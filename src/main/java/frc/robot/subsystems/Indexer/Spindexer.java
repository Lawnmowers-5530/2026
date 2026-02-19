package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    TalonFX spindexer = new TalonFX(26, "canivore");
    TalonFX kicker = new TalonFX(27, "canivore");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();


    public Spindexer() {

        this.spindexer.getConfigurator().apply(spindexerConfig);
        this.kicker.getConfigurator().apply(kickerConfig);
    };

    public void spinKick() {
        this.spindexer.set(0.5);
        this.kicker.set(1);
    }

    public void stopSpinKick() {
        this.spindexer.set(0);
        this.kicker.set(0);
    }

    public Command spinKickCommand() {
        return Commands.runOnce(() -> {this.spinKick();}, this);
    }

    public Command stopSpinKickCommand() {
        return Commands.runOnce(() -> {this.stopSpinKick();});
    }
}
