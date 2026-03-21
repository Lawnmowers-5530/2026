package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SpindexerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Spindexer extends SubsystemBase {
    TalonFX spindexer = new TalonFX(SpindexerConstants.spindexerMotorPort, RobotConstants.canivoreBus);
    TalonFX kicker = new TalonFX(SpindexerConstants.kickerMotorPort, RobotConstants.canivoreBus);

    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    VoltageOut spindexerControl = new VoltageOut(0);
    VoltageOut kickerControl = new VoltageOut(0);

    Tunables tunables;

    public Spindexer() {
        this.spindexer.getConfigurator().apply(spindexerConfig);
        this.kicker.getConfigurator().apply(kickerConfig);
        spindexerControl.EnableFOC = true;
        kickerControl.EnableFOC = true;

        tunables = new Tunables();
    }

    @Override
    public void periodic() {
        tunables.updateDashboardConfig();
    }

    public Command smartDashboardSpindexerCommand(String kickerSpeedKey, String spindexerSpeedKey) {
        SmartDashboard.putNumber(kickerSpeedKey, 0);
        SmartDashboard.putNumber(spindexerSpeedKey, 0);
        return Commands.run(()->{
            this.spindexer.setControl(new VoltageOut(SmartDashboard.getNumber(spindexerSpeedKey, 0)));
            this.kicker.setControl(new VoltageOut(SmartDashboard.getNumber(kickerSpeedKey, 0)));
        }, this);
    }

    public void spin() {
        spindexerControl.Output = SpindexerConstants.spindexerForwardSpeed;
        this.spindexer.setControl(spindexerControl);
    }

    public void spinKick() {
        spindexerControl.Output = SpindexerConstants.spindexerForwardSpeed;
        kickerControl.Output = SpindexerConstants.kickerForwardSpeed;
        this.spindexer.setControl(spindexerControl);
        this.kicker.setControl(kickerControl);
    }

    public void spinKickFast() {
        spindexerControl.Output = SpindexerConstants.spindexerFastSpeed;
        kickerControl.Output = SpindexerConstants.kickerFastSpeed;
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

    public Command
    spinKickCommand() {
        return new RunCommand(() -> {
            this.spinKick();
        }, this);
    }

    public Command spinKickFastCommand() {
        return new RunCommand(() -> {
            this.spinKickFast();
        }, this);
    }

    public Command spinCommand() {
        return new RunCommand(() -> {
            this.spin();
        }, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            this.stopSpinKick();
        });
    }

    public Command reverseCommand() {
        return new RunCommand(() -> {
            this.reverse();
        }, this);
    }

    public Command jitter() {
        return new ParallelDeadlineGroup(
                new WaitCommand(IndexerConstants.jitterTime),
                this.spinCommand())
                .andThen(
                        new ParallelDeadlineGroup(
                                new WaitCommand(IndexerConstants.jitterTime),
                                this.reverseCommand()));
    }

    private static class Tunables {
        private final LoggedNetworkBoolean tuningEnabled = new LoggedNetworkBoolean(SpindexerConstants.dashboardPath + "/tuningEnabled");

        private final LoggedNetworkNumber
            spindexerForwardSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/spindexerForwardSpeed", SpindexerConstants.spindexerForwardSpeed),
            spindexerFastSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/spindexerFastSpeed", SpindexerConstants.spindexerFastSpeed),
            spindexerReverseSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/spindexerReverseSpeed", SpindexerConstants.spindexerReverseSpeed),
            kickerForwardSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/kickerForwardSpeed", SpindexerConstants.kickerForwardSpeed),
            kickerFastSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/kickerFastSpeed", SpindexerConstants.kickerFastSpeed),
            kickerReverseSpeed = new LoggedNetworkNumber(SpindexerConstants.dashboardPath + "/kickerReverseSpeed", SpindexerConstants.kickerReverseSpeed);

        private void updateDashboardConfig() {
            if (!tuningEnabled.get()) return;

            // Directly update the constants from the dashboard values (no conditional checks)
            SpindexerConstants.spindexerForwardSpeed = spindexerForwardSpeed.get();
            SpindexerConstants.spindexerFastSpeed = spindexerFastSpeed.get();
            SpindexerConstants.spindexerReverseSpeed = spindexerReverseSpeed.get();

            SpindexerConstants.kickerForwardSpeed = kickerForwardSpeed.get();
            SpindexerConstants.kickerFastSpeed = kickerFastSpeed.get();
            SpindexerConstants.kickerReverseSpeed = kickerReverseSpeed.get();
        }
    }
}
