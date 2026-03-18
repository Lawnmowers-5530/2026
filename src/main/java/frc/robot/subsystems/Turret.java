package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TurretConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class Turret extends SubsystemBase {

    // group related private fields with the same visibility modifier
    private TalonFX m_yaw, m_pitch, m_flywheel;
    private TalonFXConfiguration yawConfig = new TalonFXConfiguration(), pitchConfig = new TalonFXConfiguration(),
            flywheelConfig = new TalonFXConfiguration();
    private MotionMagicVoltage yawControl, pitchControl;
    private VelocityVoltage flywheelControl;
    private VoltageOut flywheelSysIdControl;
    private StatusSignal<Angle> flywheelPositionSignal, pitchPositionSignal, yawPositionSignal;
    private StatusSignal<AngularVelocity> flywheelVelocitySignal, yawVelocitySignal;
    private StatusSignal<Voltage> flywheelVoltageStatusSignal, yawVoltageSignal;
    private SysIdRoutine flywheelRoutine;
    private Tunables tunables;

    public Turret() {
        CommandScheduler.getInstance().registerSubsystem(this);

        var slot0yawConfig = yawConfig.Slot0;
        slot0yawConfig.kS = TurretConstants.yaw_kS; // Add 0.25 V output to overcome static friction
        slot0yawConfig.kV = TurretConstants.yaw_kV; // A velocity target of 1 rps results in 0.12 V output
        slot0yawConfig.kA = TurretConstants.yaw_kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0yawConfig.kP = TurretConstants.yaw_kP; // A position error of 2.5 rotations results in 12 V output
        slot0yawConfig.kI = TurretConstants.yaw_kI; // no output for integrated error
        slot0yawConfig.kD = TurretConstants.yaw_kD; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigYaw = yawConfig.MotionMagic;
        motionMagicConfigYaw.MotionMagicCruiseVelocity = TurretConstants.yawMotionMagicCruiseVelocity;
        motionMagicConfigYaw.MotionMagicAcceleration = TurretConstants.yawMotionMagicAcceleration;
        motionMagicConfigYaw.MotionMagicJerk = TurretConstants.yawMotionMagicJerk;

        var slot0pitchConfig = pitchConfig.Slot0;
        slot0pitchConfig.kS = TurretConstants.pitch_kS; // Add 0.25 V output to overcome static friction
        slot0pitchConfig.kV = TurretConstants.pitch_kV; // A velocity target of 1 rps results in 0.12 V output
        slot0pitchConfig.kA = TurretConstants.pitch_kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0pitchConfig.kP = TurretConstants.pitch_kP;// 6; // A position error of 2.5 rotations results in 12 V output
        slot0pitchConfig.kI = TurretConstants.pitch_kI; // no output for integrated error
        slot0pitchConfig.kD = TurretConstants.pitch_kD; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigpitch = pitchConfig.MotionMagic;
        motionMagicConfigpitch.MotionMagicCruiseVelocity = TurretConstants.pitchMotionMagicCruiseVelocity;
        motionMagicConfigpitch.MotionMagicAcceleration = TurretConstants.pitchMotionMagicAcceleration;
        motionMagicConfigpitch.MotionMagicJerk = TurretConstants.pitchMotionMagicJerk;

        var slot0flywheelConfig = flywheelConfig.Slot0;
        slot0flywheelConfig.kS = TurretConstants.flywheel_kS; // Add 0.2 V output to overcome static friction
        slot0flywheelConfig.kV = TurretConstants.flywheel_kV; // A velocity target of 1 rps results in 0.1 V output
        slot0flywheelConfig.kA = TurretConstants.flywheel_kA; // An acceleration of 1 rps/s requires 0.005 V output
        slot0flywheelConfig.kP = TurretConstants.flywheel_kP; // A position error of 2.5 rotations results in 12 V
                                                              // output
        slot0flywheelConfig.kI = TurretConstants.flywheel_kI; // no output for integrated error
        slot0flywheelConfig.kD = TurretConstants.flywheel_kD; // A velocity error of 1 rps results in 0.05 V output

        var slot1FlywheelConfig = flywheelConfig.Slot1;
        slot1FlywheelConfig.kS = TurretConstants.flywheel_kS;
        slot1FlywheelConfig.kV = TurretConstants.flywheel_kV;
        slot1FlywheelConfig.kA = TurretConstants.flywheel_kA;
        slot1FlywheelConfig.kP = 1;
        slot1FlywheelConfig.kI = TurretConstants.flywheel_kI; // no output for integrated error
        slot1FlywheelConfig.kD = TurretConstants.flywheel_kD; // A velocity error of 1 rps results in 0.05 V output

        yawConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_yaw = new TalonFX(21, TurretConstants.canBus);
        this.m_yaw.getConfigurator().apply(yawConfig);
        this.yawControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);

        pitchConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO ensure correct direction

        this.m_pitch = new TalonFX(22, TurretConstants.canBus);
        this.m_pitch.getConfigurator().apply(pitchConfig);
        this.pitchControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        this.m_pitch.setControl(pitchControl);

        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.m_flywheel = new TalonFX(23, TurretConstants.canBus);
        this.m_flywheel.getConfigurator().apply(flywheelConfig);
        this.flywheelControl = new VelocityVoltage(0).withEnableFOC(true);

        this.m_flywheel.optimizeBusUtilization();
        this.m_pitch.optimizeBusUtilization();
        this.m_yaw.optimizeBusUtilization();

        this.flywheelSysIdControl = new VoltageOut(0).withEnableFOC(true);
        this.flywheelVoltageStatusSignal = this.m_flywheel.getMotorVoltage();
        this.flywheelPositionSignal = this.m_flywheel.getPosition();
        this.flywheelVelocitySignal = this.m_flywheel.getVelocity();
        this.pitchPositionSignal = this.m_pitch.getPosition();

        this.flywheelVoltageStatusSignal.setUpdateFrequency(1000);
        this.flywheelPositionSignal.setUpdateFrequency(1000);
        this.flywheelVelocitySignal.setUpdateFrequency(1000);

        this.yawPositionSignal = this.m_yaw.getPosition();
        this.yawVelocitySignal = this.m_yaw.getVelocity();
        this.yawVoltageSignal = this.m_yaw.getMotorVoltage();

        this.yawPositionSignal.setUpdateFrequency(1000);
        this.yawVelocitySignal.setUpdateFrequency(1000);
        this.yawVoltageSignal.setUpdateFrequency(1000);
        // SmartDashboard.putNumber(TurretConstants.dashboardPath + "/yaw-kp", 3.5);
        // SmartDashboard.putNumber(TurretConstants.dashboardPath + "/yaw-ks",
        // TurretConstants.yaw_kS);
        // SmartDashboard.putNumber(TurretConstants.dashboardPath + "/yaw-kv",
        // TurretConstants.yaw_kV);
        SysIdRoutine.Mechanism flywheelMechanism = new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this);

        SysIdRoutine.Config flywheelConfig = new SysIdRoutine.Config(
                Volts.per(Second).of(0.5), // 1 V/s voltage ramp
                Volts.of(2),
                Second.of(20),
                (state) -> {
                    SignalLogger.writeString("flywheelState", state.toString());
                });

        this.flywheelRoutine = new SysIdRoutine(flywheelConfig, flywheelMechanism);

        tunables = new Tunables();
    }

    public void setYaw(Rotation2d pos) {
        // clamp input
        pos = Rotation2d.fromDegrees(
                MathUtil.clamp(pos.getDegrees(), TurretConstants.turretYawMin, TurretConstants.turretYawMax));
        // convert angle to controller position units (radians here as an example)
        Rotation2d targetPosition = Rotation2d.fromRadians(MathUtil.angleModulus(pos.getRadians()))
                .plus(TurretConstants.turretOffset);

        this.yawControl.Position = targetPosition.times(TurretConstants.motorToYawRot).getRotations();
        this.m_yaw.setControl(this.yawControl);

        SmartDashboard.putString("Encoder Pos", this.m_yaw.getPosition().toString());
    }

    public Command smartDashboardTurretCommand(String pitchKey, String speedRPSKey, String yawKey) {
        SmartDashboard.putNumber(yawKey, TurretConstants.turretOffset.times(-1).getDegrees());
        SmartDashboard.putNumber(speedRPSKey, 0);
        SmartDashboard.putNumber(pitchKey, 72);
        return this.setTurretStateCommand(() -> {
            return new TurretState(
                    Rotation2d.fromDegrees(
                            SmartDashboard.getNumber(yawKey, TurretConstants.turretOffset.times(-1).getDegrees())),
                    Rotation2d.fromDegrees(SmartDashboard.getNumber(pitchKey, 72)),
                    SmartDashboard.getNumber(speedRPSKey, 0));
        });
    }

    public void setPitch(Rotation2d pos) {
        pos = pos.minus(TurretConstants.pitchZeroAngle);
        SmartDashboard.putNumber("pospreclamp", pos.getDegrees());
        double targetPosition = pos.getDegrees() * TurretConstants.motorRotToPitchDeg;
        this.pitchControl.Position = targetPosition;
        this.m_pitch.setControl(this.pitchControl);
    }

    boolean flag = false;

    public void setFlywheelSpeed(double speed) {
        SmartDashboard.putNumber("Turret/ Real Velo", this.m_flywheel.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/ Applied Voltage", this.m_flywheel.getMotorVoltage().getValueAsDouble());
        if (m_flywheel.getVelocity().getValue().in(RotationsPerSecond) < speed * 0.7) {
            m_flywheel.setControl(flywheelControl.withVelocity(speed).withSlot(1));
            return;
        }
        m_flywheel.setControl(flywheelControl.withVelocity(speed).withSlot(0));

    }

    public TurretState getTurretState() {
        return new TurretState(
                Rotation2d.fromRotations(this.m_yaw.getPosition().getValueAsDouble() / TurretConstants.motorToYawRot)
                        .minus(TurretConstants.turretOffset),
                Rotation2d
                        .fromRotations(
                                this.m_pitch.getPosition().getValueAsDouble() / TurretConstants.motorRotToPitchDeg)
                        .plus(TurretConstants.pitchZeroAngle),
                this.m_flywheel.getVelocity().getValueAsDouble() * TurretConstants.motorToFlywheelRot);
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
        tunables.updateDashboardConfig();

    }

    public void setVoltage(Voltage voltage) {
        this.flywheelSysIdControl = this.flywheelSysIdControl.withOutput(voltage);
        this.m_yaw.setControl(this.flywheelSysIdControl);
    }

    public SysIdRoutine getSysIdRoutine() {
        return flywheelRoutine;
    }

    @AutoLogOutput(key = TurretConstants.dashboardPath + "/pitchPosition")
    public double getPitchPosition() {
        return this.pitchPositionSignal.refresh().getValueAsDouble();
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

    private class Tunables {
        // Tuning enable
        private final LoggedNetworkBoolean tuningEnabled = new LoggedNetworkBoolean(
                TurretConstants.dashboardPath + "/tuningEnabled", false);

        // Grouped LoggedNetworkNumber declarations for motor configs and non-motor
        // constants
        private final LoggedNetworkNumber yaw_kS = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kS",
                TurretConstants.yaw_kS),
                yaw_kV = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kV", TurretConstants.yaw_kV),
                yaw_kA = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kA", TurretConstants.yaw_kA),
                yaw_kP = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kP", TurretConstants.yaw_kP),
                yaw_kI = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kI", TurretConstants.yaw_kI),
                yaw_kD = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/kD", TurretConstants.yaw_kD),
                yaw_cruise = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/cruise",
                        TurretConstants.yawMotionMagicCruiseVelocity),
                yaw_acc = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/acc",
                        TurretConstants.yawMotionMagicAcceleration),
                yaw_jerk = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/yaw/jerk",
                        TurretConstants.yawMotionMagicJerk),

                pitch_kS = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kS",
                        TurretConstants.pitch_kS),
                pitch_kV = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kV",
                        TurretConstants.pitch_kV),
                pitch_kA = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kA",
                        TurretConstants.pitch_kA),
                pitch_kP = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kP",
                        TurretConstants.pitch_kP),
                pitch_kI = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kI",
                        TurretConstants.pitch_kI),
                pitch_kD = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/kD",
                        TurretConstants.pitch_kD),
                pitch_cruise = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/cruise",
                        TurretConstants.pitchMotionMagicCruiseVelocity),
                pitch_acc = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/acc",
                        TurretConstants.pitchMotionMagicAcceleration),
                pitch_jerk = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/pitch/jerk",
                        TurretConstants.pitchMotionMagicJerk),

                fly_kS = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kS",
                        TurretConstants.flywheel_kS),
                fly_kV = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kV",
                        TurretConstants.flywheel_kV),
                fly_kA = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kA",
                        TurretConstants.flywheel_kA),
                fly_kP = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kP",
                        TurretConstants.flywheel_kP),
                fly_kI = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kI",
                        TurretConstants.flywheel_kI),
                fly_kD = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/flywheel/kD",
                        TurretConstants.flywheel_kD),

                // non-motor tunables (updated unconditionally)
                motorToYawRot = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/motorToYawRot",
                        TurretConstants.motorToYawRot),
                motorRotToPitchDeg = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/motorRotToPitchDeg",
                        TurretConstants.motorRotToPitchDeg),
                motorToFlywheelRot = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/motorToFlywheelRot",
                        TurretConstants.motorToFlywheelRot),
                sysIdRamp = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/sysIdRampRate",
                        TurretConstants.sysIdRampRate),
                sysIdStep = new LoggedNetworkNumber(TurretConstants.dashboardPath + "/sysIdDynamicStepVoltage",
                        TurretConstants.sysIdDynamicStepVoltage);

        private void updateDashboardConfig() {
            // System.out.println("updating dashboard config");
            if (!tuningEnabled.get())
                return;
            // System.out.println("passed tuning enabled check");

            Slot0Configs yawSlot = yawConfig.Slot0;
            MotionMagicConfigs yawMM = yawConfig.MotionMagic;

            Slot0Configs pitchSlot = pitchConfig.Slot0;
            MotionMagicConfigs pitchMM = pitchConfig.MotionMagic;

            Slot0Configs flySlot = flywheelConfig.Slot0;

            boolean changed = false;

            double newYawKS = yaw_kS.get();

            if (yawSlot.kS != newYawKS) {
                yawSlot.kS = newYawKS;
                changed = true;
                TurretConstants.yaw_kS = newYawKS;
            }
            double newYawKV = yaw_kV.get();
            if (yawSlot.kV != newYawKV) {
                yawSlot.kV = newYawKV;
                changed = true;
                TurretConstants.yaw_kV = newYawKV;
            }
            double newYawKA = yaw_kA.get();
            if (yawSlot.kA != newYawKA) {
                yawSlot.kA = newYawKA;
                changed = true;
                TurretConstants.yaw_kA = newYawKA;
            }
            double newYawKP = yaw_kP.get();
            if (yawSlot.kP != newYawKP) {
                System.out.println("new yaw kp: " + newYawKP);
                yawSlot.kP = newYawKP;
                changed = true;
                TurretConstants.yaw_kP = newYawKP;
            }
            double newYawKI = yaw_kI.get();
            if (yawSlot.kI != newYawKI) {
                yawSlot.kI = newYawKI;
                changed = true;
                TurretConstants.yaw_kI = newYawKI;
            }
            double newYawKD = yaw_kD.get();
            if (yawSlot.kD != newYawKD) {
                yawSlot.kD = newYawKD;
                changed = true;
                TurretConstants.yaw_kD = newYawKD;
            }

            double newYawCruise = yaw_cruise.get();
            if (yawMM.MotionMagicCruiseVelocity != newYawCruise) {
                yawMM.MotionMagicCruiseVelocity = newYawCruise;
                changed = true;
                TurretConstants.yawMotionMagicCruiseVelocity = newYawCruise;
            }
            double newYawAcc = yaw_acc.get();
            if (yawMM.MotionMagicAcceleration != newYawAcc) {
                yawMM.MotionMagicAcceleration = newYawAcc;
                changed = true;
                TurretConstants.yawMotionMagicAcceleration = newYawAcc;
            }
            double newYawJerk = yaw_jerk.get();
            if (yawMM.MotionMagicJerk != newYawJerk) {
                yawMM.MotionMagicJerk = newYawJerk;
                changed = true;
                TurretConstants.yawMotionMagicJerk = newYawJerk;
            }

            double newPitchKS = pitch_kS.get();
            if (pitchSlot.kS != newPitchKS) {
                pitchSlot.kS = newPitchKS;
                changed = true;
                TurretConstants.pitch_kS = newPitchKS;
            }
            double newPitchKV = pitch_kV.get();
            if (pitchSlot.kV != newPitchKV) {
                pitchSlot.kV = newPitchKV;
                changed = true;
                TurretConstants.pitch_kV = newPitchKV;
            }
            double newPitchKA = pitch_kA.get();
            if (pitchSlot.kA != newPitchKA) {
                pitchSlot.kA = newPitchKA;
                changed = true;
                TurretConstants.pitch_kA = newPitchKA;
            }
            double newPitchKP = pitch_kP.get();
            if (pitchSlot.kP != newPitchKP) {
                pitchSlot.kP = newPitchKP;
                changed = true;
                TurretConstants.pitch_kP = newPitchKP;
            }
            double newPitchKI = pitch_kI.get();
            if (pitchSlot.kI != newPitchKI) {
                pitchSlot.kI = newPitchKI;
                changed = true;
                TurretConstants.pitch_kI = newPitchKI;
            }
            double newPitchKD = pitch_kD.get();
            if (pitchSlot.kD != newPitchKD) {
                pitchSlot.kD = newPitchKD;
                changed = true;
                TurretConstants.pitch_kD = newPitchKD;
            }

            double newPitchCruise = pitch_cruise.get();
            if (pitchMM.MotionMagicCruiseVelocity != newPitchCruise) {
                pitchMM.MotionMagicCruiseVelocity = newPitchCruise;
                changed = true;
                TurretConstants.pitchMotionMagicCruiseVelocity = newPitchCruise;
            }
            double newPitchAcc = pitch_acc.get();
            if (pitchMM.MotionMagicAcceleration != newPitchAcc) {
                pitchMM.MotionMagicAcceleration = newPitchAcc;
                changed = true;
                TurretConstants.pitchMotionMagicAcceleration = newPitchAcc;
            }
            double newPitchJerk = pitch_jerk.get();
            if (pitchMM.MotionMagicJerk != newPitchJerk) {
                pitchMM.MotionMagicJerk = newPitchJerk;
                changed = true;
                TurretConstants.pitchMotionMagicJerk = newPitchJerk;
            }

            double newFlyKS = fly_kS.get();
            if (flySlot.kS != newFlyKS) {
                flySlot.kS = newFlyKS;
                changed = true;
                TurretConstants.flywheel_kS = newFlyKS;
            }
            double newFlyKV = fly_kV.get();
            if (flySlot.kV != newFlyKV) {
                flySlot.kV = newFlyKV;
                changed = true;
                TurretConstants.flywheel_kV = newFlyKV;
            }
            double newFlyKA = fly_kA.get();
            if (flySlot.kA != newFlyKA) {
                flySlot.kA = newFlyKA;
                changed = true;
                TurretConstants.flywheel_kA = newFlyKA;
            }
            double newFlyKP = fly_kP.get();
            if (flySlot.kP != newFlyKP) {
                flySlot.kP = newFlyKP;
                changed = true;
                TurretConstants.flywheel_kP = newFlyKP;
            }
            double newFlyKI = fly_kI.get();
            if (flySlot.kI != newFlyKI) {
                flySlot.kI = newFlyKI;
                changed = true;
                TurretConstants.flywheel_kI = newFlyKI;
            }
            double newFlyKD = fly_kD.get();
            if (flySlot.kD != newFlyKD) {
                flySlot.kD = newFlyKD;
                changed = true;
                TurretConstants.flywheel_kD = newFlyKD;
            }

            // non motor constants: write unconditionally
            TurretConstants.motorToYawRot = motorToYawRot.get();
            TurretConstants.motorRotToPitchDeg = motorRotToPitchDeg.get();
            TurretConstants.motorToFlywheelRot = motorToFlywheelRot.get();
            TurretConstants.sysIdRampRate = sysIdRamp.get();
            TurretConstants.sysIdDynamicStepVoltage = sysIdStep.get();

            if (changed) {
                System.out.print("Applied Configs");
                yawConfig = yawConfig.withSlot0(yawSlot).withMotionMagic(yawMM);
                pitchConfig = pitchConfig.withSlot0(pitchSlot).withMotionMagic(pitchMM);
                flywheelConfig = flywheelConfig.withSlot0(flySlot);
                m_yaw.getConfigurator().apply(yawConfig);
                m_pitch.getConfigurator().apply(pitchConfig);
                m_flywheel.getConfigurator().apply(flywheelConfig);
            }
        }
    }
}
