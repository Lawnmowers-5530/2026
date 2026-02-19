package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase{
    TalonFX spindexer = new TalonFX(SpindexerConstants.SPINDEXER_MOTOR_PORT, "canivore");
    TalonFX kicker = new TalonFX(SpindexerConstants.KICKER_MOTOR_PORT, "canivore");

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    DutyCycleOut spindexerControl = new DutyCycleOut(0);
    DutyCycleOut kickerControl = new DutyCycleOut(0);



    public Spindexer() {
        this.spindexer.getConfigurator().apply(spindexerConfig);
        this.kicker.getConfigurator().apply(kickerConfig);
        spindexerControl.EnableFOC = true;
        kickerControl.EnableFOC = true;
    };

    public void spinKick() {
        spindexerControl.Output = SpindexerConstants.spindexerForwardSpeed;
        kickerControl.Output = SpindexerConstants.kickerForwardSpeed;
        this.spindexer.setControl(spindexerControl);
        this.kicker.setControl(kickerControl);
    }

    public void stopSpinKick() {
        this.spindexer.stopMotor();
        this.kicker.stopMotor();
    }

    public void reverse() {
        spindexerControl.Output = SpindexerConstants.spindexerReverseSpeed;
        kickerControl.Output = SpindexerConstants.kickerReverseSpeed;
        this.spindexer.setControl(spindexerControl);
        this.kicker.setControl(kickerControl);
    }

    public Command spinKickCommand() {
        return Commands.runOnce(() -> {this.spinKick();}, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {this.stopSpinKick();});
    }

    public Command reverseCommand() {
        return Commands.runOnce(() -> {this.reverse();}, this);
    }
}
