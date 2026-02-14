package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.LauncherConstants;

import static edu.wpi.first.units.Units.*;

public class LauncherFlywheel extends SubsystemBase {
    TalonFX motor;

    VelocityVoltage pidControlRequest;
    VoltageOut sysIdControlRequest;

    StatusSignal<Angle> positionSignal;
    StatusSignal<AngularVelocity> velocitySignal;
    StatusSignal<Voltage> appliedOutputSignal;

    SysIdRoutine sysIdRoutine;

    public LauncherFlywheel(int id) {
        this.motor = new TalonFX(id);
        this.positionSignal = motor.getPosition();
        this.velocitySignal = motor.getVelocity();
        this.appliedOutputSignal = motor.getMotorVoltage();

        var slotConfig = new Slot0Configs()
                .withKV(LauncherConstants.kV)
                .withKA(LauncherConstants.kA)
                .withKS(LauncherConstants.kS)
                .withKP(LauncherConstants.kP)
                .withKI(LauncherConstants.kI)
                .withKD(LauncherConstants.kD);

        var talonFxConfig = new TalonFXConfiguration()
                .withSlot0(slotConfig);

        motor.getConfigurator().apply(talonFxConfig);

        pidControlRequest.withEnableFOC(true);
        sysIdControlRequest.withEnableFOC(true);

        var sysIdMechanism = new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this
        );

        var sysIdConfig = new SysIdRoutine.Config(
                Volts.per(Second).of(LauncherConstants.sysIdRampRate),
                Volts.of(LauncherConstants.sysIdDynamicStepVoltage),
                Second.of(LauncherConstants.sysIdTimeout),
                (state) -> SignalLogger.writeString("state", state.toString())
        );

        this.sysIdRoutine = new SysIdRoutine(sysIdConfig, sysIdMechanism);
    }

    public void setVelocity(double velocity) {
        this.pidControlRequest.withVelocity(velocity);
        motor.setControl(pidControlRequest);
    }

    public void setVoltage(Voltage voltage) {
        this.sysIdControlRequest.withOutput(voltage);
        motor.setControl(sysIdControlRequest);
    }

    public void setVoltage(double voltage) {
        this.sysIdControlRequest.withOutput(voltage);
        motor.setControl(sysIdControlRequest);
    }

    public Angle getPosition() {
        return positionSignal.refresh().getValue();
    }

    public AngularVelocity getVelocity() {
        return velocitySignal.refresh().getValue();
    }

    public Voltage getAppliedOutput() {
        return appliedOutputSignal.refresh().getValue();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
