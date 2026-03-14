package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TurretConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class Turret extends SubsystemBase {

    private TalonFX m_yaw;
    private TalonFXConfiguration yawConfig = new TalonFXConfiguration();
    private MotionMagicVoltage yawControl;
    private TalonFX m_pitch;
    private TalonFXConfiguration pitchConfig = new TalonFXConfiguration();
    private MotionMagicVoltage pitchControl;
    private TalonFX m_flywheel;
    private TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    private VelocityVoltage flywheelControl;
    private VoltageOut flywheelSysIdControl;
    private StatusSignal<Angle> flywheelPositionSignal;
    private StatusSignal<AngularVelocity> flywheelVelocitySignal;
    private StatusSignal<Voltage> voltageStatusSignal;
    private SysIdRoutine flywheelRoutine;
    public Turret() {
        CommandScheduler.getInstance().registerSubsystem(this);

        var slot0yawConfig = yawConfig.Slot0;
        slot0yawConfig.kS = 0.4; // Add 0.25 V output to overcome static friction
        slot0yawConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0yawConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0yawConfig.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0yawConfig.kI = 0; // no output for integrated error
        slot0yawConfig.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigYaw = yawConfig.MotionMagic;
        motionMagicConfigYaw.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigYaw.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigYaw.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        var slot0pitchConfig = pitchConfig.Slot0;
        slot0pitchConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0pitchConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0pitchConfig.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0pitchConfig.kP = 7;//6; // A position error of 2.5 rotations results in 12 V output
        slot0pitchConfig.kI = 0; // no output for integrated error
        slot0pitchConfig.kD = 0; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigpitch = pitchConfig.MotionMagic;
        motionMagicConfigpitch.MotionMagicCruiseVelocity = 16; // Target cruise velocity of 80 rps
        motionMagicConfigpitch.MotionMagicAcceleration = 300; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigpitch.MotionMagicJerk = 4000; // Target jerk of 1600 rps/s/s (0.1 seconds)

        var slot0flywheelConfig = flywheelConfig.Slot0;
        slot0flywheelConfig.kS = TurretConstants.kS; // Add 0.2 V output to overcome static friction
        slot0flywheelConfig.kV = TurretConstants.kV; // A velocity target of 1 rps results in 0.1 V output
        slot0flywheelConfig.kA = TurretConstants.kA; // An acceleration of 1 rps/s requires 0.005 V output
        slot0flywheelConfig.kP = TurretConstants.kP; // A position error of 2.5 rotations results in 12 V output
        slot0flywheelConfig.kI = 0; // no output for integrated error
        slot0flywheelConfig.kD = 0; // A velocity error of 1 rps results in 0.05 V output

        yawConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_yaw = new TalonFX(21, TurretConstants.canBus);
        this.m_yaw.getConfigurator().apply(yawConfig);
        this.yawControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        //this.m_yaw.setControl(yawControl);

        pitchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO ensure correct direction

        this.m_pitch = new TalonFX(22, TurretConstants.canBus);
        this.m_pitch.getConfigurator().apply(pitchConfig);
        this.pitchControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        this.m_pitch.setControl(pitchControl);

        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_flywheel = new TalonFX(23, TurretConstants.canBus);
        this.m_flywheel.getConfigurator().apply(flywheelConfig);
        this.flywheelControl = new VelocityVoltage(0);
        this.m_flywheel.setControl(flywheelControl);

        this.flywheelSysIdControl = new VoltageOut(0).withEnableFOC(true);
        this.voltageStatusSignal = this.m_flywheel.getMotorVoltage();
        this.flywheelPositionSignal = this.m_flywheel.getPosition();
        this.flywheelVelocitySignal = this.m_flywheel.getVelocity();
        this.m_flywheel.optimizeBusUtilization();
        this.voltageStatusSignal.setUpdateFrequency(1000);
        this.flywheelPositionSignal.setUpdateFrequency(1000);
        this.flywheelVelocitySignal.setUpdateFrequency(1000);

        SysIdRoutine.Mechanism flywheelMechanism = new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this
        );

        SysIdRoutine.Config flywheelConfig = new SysIdRoutine.Config(
                Volts.per(Second).of(0.6), // 1 V/s voltage ramp
                Volts.of(3),
                Second.of(4),
                (state) -> {
                    SignalLogger.writeString("flywheelState", state.toString());
                }
        );

        this.flywheelRoutine = new SysIdRoutine(flywheelConfig, flywheelMechanism);
    }

    public void setYaw(Rotation2d pos) {
        // convert angle to controller position units (radians here as an example)
        Rotation2d targetPosition = Rotation2d.fromRadians(MathUtil.angleModulus(pos.getRadians())).plus(TurretConstants.turretOffset);

        this.yawControl.Position = targetPosition.times(TurretConstants.motorToYawRot).getRotations();
        this.m_yaw.setControl(this.yawControl);

        SmartDashboard.putString("Encoder Pos", this.m_yaw.getPosition().toString());
    }
    public Command smartDashboardTurretCommand(String pitchKey, String speedRPSKey, String yawKey) {
        SmartDashboard.putNumber(yawKey, 0);
        SmartDashboard.putNumber(speedRPSKey, 0);
        SmartDashboard.putNumber(pitchKey, 72);
        return this.setTurretStateCommand(()->{return new TurretState(Rotation2d.fromDegrees(SmartDashboard.getNumber(yawKey, 0)), Rotation2d.fromDegrees(SmartDashboard.getNumber(pitchKey, 72)), SmartDashboard.getNumber(speedRPSKey, 0));});
    }

    public void setPitch(Rotation2d pos) {
        pos = pos.minus(TurretConstants.pitchZeroAngle);
        SmartDashboard.putNumber("pospreclamp", pos.getDegrees());
        double targetPosition = pos.getDegrees() * TurretConstants.motorRotToPitchDeg;
        this.pitchControl.Position = targetPosition;
        this.m_pitch.setControl(this.pitchControl);
    }

    public void setFlywheelSpeed(double speed) {
        // convert speed to controller velocity units (rps here as an example)
        double targetVelocity = speed;//LauncherConstants.VelocityToRPS.get(speed); // adjust by gear ratio / sensor units as needed
        if (speed == 0) {
            targetVelocity = 0;
        }
        this.flywheelControl.Velocity = targetVelocity;
        this.m_flywheel.setControl(this.flywheelControl);
    }

    public TurretState getTurretState() {
        return new TurretState(
                Rotation2d.fromRotations(this.m_yaw.getPosition().getValueAsDouble() / TurretConstants.motorToYawRot).minus(TurretConstants.turretOffset),
                Rotation2d.fromRotations(this.m_pitch.getPosition().getValueAsDouble() / TurretConstants.motorRotToPitchDeg).plus(TurretConstants.pitchZeroAngle),
                this.m_flywheel.getVelocity().getValueAsDouble() * TurretConstants.motorToFlywheelRot
        );
    }

    public void setTurretState(TurretState state) {
        this.setYaw(state.yaw);
        this.setPitch(state.pitch);
        this.setFlywheelSpeed(state.flywheelSpeed);
    }

    public void zeroYaw() {
        this.m_yaw.setPosition(0);
    }

    public void zeroPitch() {
        this.m_pitch.setPosition(0);
    }

    public void fourRotations() {
        this.m_yaw.setPosition(0);
        this.yawControl.Position = 0.25 * TurretConstants.motorToYawRot;
        this.m_yaw.setControl(this.yawControl);
    }

    public Command setFlywheelSpeedCommand(double speed) {
        return new InstantCommand(() -> {
            this.setFlywheelSpeed(speed);
        }, this);
    }

    public Command setPitchCommand(Rotation2d angle) {
        return new InstantCommand(() -> {
            this.setPitch(angle);
        }, this);
    }

    public Command setTurretStateCommand(Supplier<TurretState> state) {
        SmartDashboard.putString("goalTurretState", state.get().toString());
        return new RunCommand(() -> {
            this.setTurretState(state.get());
        }, this);
    }

    public void periodic() {
        SmartDashboard.putNumber("pitch encoder value", this.m_pitch.getPosition().getValueAsDouble());
    }

    public void setVoltage(Voltage voltage) {
        this.flywheelSysIdControl = this.flywheelSysIdControl.withOutput(voltage);
        this.m_yaw.setControl(this.flywheelSysIdControl);
    }

    public SysIdRoutine getSysIdRoutine() {
        return flywheelRoutine;
    }

    public static class TurretState {
        public Rotation2d yaw;
        public Rotation2d pitch;
        public double flywheelSpeed;

        public TurretState(Rotation2d yaw, Rotation2d pitch, double flywheelSpeed) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.flywheelSpeed = flywheelSpeed;
        }

        @Override
        public String toString() {
            return String.format("TurretState(yaw=%.2f deg, pitch=%.2f deg, flywheelSpeed=%.2f)",
                    yaw.getDegrees(), pitch.getDegrees(), flywheelSpeed);
        }

        public TurretState rotateBy(Rotation2d rotation) {
            this.yaw = this.yaw.plus(rotation);
            return this;
        }
    }
}
