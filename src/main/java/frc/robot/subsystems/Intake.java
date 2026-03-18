package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final TalonFX runMotor;
    private final Supplier<LinearVelocity> robotVelocity;
    private final Debouncer stallDebouncer;
    private final StatusSignal<Angle> pivotPositionSignal;
    Tunables tunables;
    @AutoLogOutput(key = IntakeConstants.dashboardPath + "/Is Extended")
    private boolean isExtended = false;
    @AutoLogOutput(key = IntakeConstants.dashboardPath + "/isStalling")
    private boolean isStalling = false;
    private TalonFXConfiguration pivotMotorConfig, runMotorConfig;

    public Intake(Supplier<LinearVelocity> robotVelocity) {
        CommandScheduler.getInstance().registerSubsystem(this);
        stallDebouncer = new Debouncer(IntakeConstants.stallDebounceSeconds, DebounceType.kRising);
        this.robotVelocity = robotVelocity;

        pivotMotor = new TalonFX(IntakeConstants.pivotMotorPort, RobotConstants.canivoreBus);
        runMotor = new TalonFX(IntakeConstants.runMotorPort, RobotConstants.canivoreBus);

        TorqueCurrentConfigs torqueCurrentConfig = new TorqueCurrentConfigs();
        runMotorConfig = new TalonFXConfiguration()
            .withTorqueCurrent(torqueCurrentConfig);
        runMotor.getConfigurator().apply(runMotorConfig);

        pivotMotorConfig = new TalonFXConfiguration();

        Slot0Configs pivotSlot0Configs = pivotMotorConfig.Slot0;
        pivotSlot0Configs.kS = IntakeConstants.pivotKS; // Add 0.25 V output to overcome static friction
        pivotSlot0Configs.kV = IntakeConstants.pivotKV; // A velocity target of 1 rps results in 0.12 V output
        pivotSlot0Configs.kA = IntakeConstants.pivotKA; // An acceleration of 1 rps/s requires 0.01 V output
        pivotSlot0Configs.kP = IntakeConstants.pivotKP; // A position error of 2.5 rotations results in 12 V output
        pivotSlot0Configs.kI = IntakeConstants.pivotKI; // no output for integrated error
        pivotSlot0Configs.kD = IntakeConstants.pivotKD; // A velocity error of 1 rps results in 0.1 V output

        pivotSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        pivotSlot0Configs.GravityArmPositionOffset = IntakeConstants.pivotGravityArmPositionOffset;
        pivotSlot0Configs.kG = IntakeConstants.pivotKG;

        MotionMagicConfigs pivotMotionMagicConfig = pivotMotorConfig.MotionMagic;
        pivotMotionMagicConfig.MotionMagicCruiseVelocity = IntakeConstants.pivotMotionMagicCruiseVelocity; // Target
        // cruise
        // velocity
        pivotMotionMagicConfig.MotionMagicAcceleration = IntakeConstants.pivotMotionMagicAcceleration; // Target
        // acceleration
        pivotMotionMagicConfig.MotionMagicJerk = IntakeConstants.pivotMotionMagicJerk; // Target jerk

        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        pivotPositionSignal = pivotMotor.getPosition();

        pivotMotor.optimizeBusUtilization();
        runMotor.optimizeBusUtilization();

        tunables = new Tunables();
    }

    @Override
    public void periodic() {
        // keep dashboard values applied when tuning
        tunables.updateDashboardConfig();
    }

    public Command toggleIntakeExtensionCommand() {
        return Commands.either(Commands.runOnce(() -> {
                isExtended = false;
                pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.tuckedEncoderPosition).withEnableFOC(true));
            }, this),
            Commands.runOnce(() -> {
                    isExtended = true;
                    pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.extendedEncoderPosition).withEnableFOC(true));
                    }, this)
                .andThen(Commands.waitUntil(this::pivotAtExtensionPosition))
                .andThen(this::applyTorqueDownward, this),
            () -> this.isExtended
        );
    }

    public Command jiggleAndRunIntakeCommand() {
        return new ParallelCommandGroup(
            Commands.repeatingSequence(
                    Commands.runOnce(() -> pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.lowerJigglePos))),
                    Commands.waitSeconds(2),
                    Commands.runOnce(() -> pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.upperJigglePos))),
                    Commands.waitSeconds(2))
                .finallyDo(() -> pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.tuckedEncoderPosition))),
            new RunCommand(this::runIntake).finallyDo(this::stopIntake)
        );
    }

    public Command jiggleIntakeCommand() {
        return Commands.repeatingSequence(
            Commands.runOnce(()->pivotMotor.setControl(new VoltageOut(-5).withEnableFOC(true)), this), 
            Commands.waitUntil(()->{return pivotMotor.getPosition().getValueAsDouble() < IntakeConstants.lowerJigglePos;}), 
            Commands.runOnce(()->pivotMotor.setControl(new VoltageOut(4).withEnableFOC(true)), this),
            Commands.waitUntil(()->{return pivotMotor.getPosition().getValueAsDouble() > IntakeConstants.upperJigglePos;}))
            .finallyDo(()->pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.extendedEncoderPosition)));
    }

    public void runIntake() {
        runMotor.setControl(new VoltageOut(IntakeConstants.runMotorVoltage));
    }

    public void stopIntake() {
        runMotor.setControl(new TorqueCurrentFOC(0));
    }

    public Command runIntakeCommand() {
        SmartDashboard.putNumber("Intake/Run Speed", 7);
        return Commands.runOnce(this::runIntake, this);

    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(this::stopIntake, this);
    }

    public Command applyTorqueDownward() {
        return Commands.runOnce(() -> {
            pivotMotor.setControl(new TorqueCurrentFOC(IntakeConstants.pivotTorqueDownwardAmps));
        }, this);
    }

    private boolean pivotAtTuckPosition() {
        return Math.abs(pivotMotor.getPosition().getValueAsDouble()
            - IntakeConstants.tuckedEncoderPosition) <= IntakeConstants.pivotPositionTolerance;
    }

    @AutoLogOutput(key = IntakeConstants.dashboardPath + "/pivotAtExtensionPosition")
    private boolean pivotAtExtensionPosition() {
        return Math.abs(pivotMotor.getPosition().getValueAsDouble()
            - IntakeConstants.extendedEncoderPosition) <= IntakeConstants.pivotPositionTolerance;
    }

    public void zeroPivot() {
        this.pivotMotor.setPosition(IntakeConstants.tuckedEncoderPosition);
    }

    public Command manualIntakeControl(Supplier<Double> runSpeed) {
        return new RunCommand(() -> {
            this.runMotor.set(runSpeed.get());
        }, this);
    }

    public Command manualPivotControl(Supplier<Double> pivotSpeed) {
        return new RunCommand(() -> {
            this.pivotMotor.set(pivotSpeed.get());
        }, this);
    }

    @AutoLogOutput(key = IntakeConstants.dashboardPath + "/pivotPosition")
    public double getPivotPosition() {
        return this.pivotPositionSignal.refresh().getValueAsDouble();
    }

    // @AutoLogOutput(key = IntakeConstants.dashboardPath + "/target")
    // too many string manipulations to make this worth it
    public String getTarget() {
        return this.pivotMotor.getAppliedControl().getControlInfo().toString();
    }

    @AutoLogOutput(key = IntakeConstants.dashboardPath + "/appliedOutput")
    public double getAppliedOutput() {
        return this.pivotMotor.get();
    }

    public class Tunables {
        private final LoggedNetworkBoolean tuningEnabled = new LoggedNetworkBoolean(
            IntakeConstants.dashboardPath + "/tuningEnabled", false);

        private final LoggedNetworkNumber pivotKS = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kS",
            IntakeConstants.pivotKS),
            pivotKV = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kV", IntakeConstants.pivotKV),
            pivotKA = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kA", IntakeConstants.pivotKA),
            pivotKP = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kP", IntakeConstants.pivotKP),
            pivotKI = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kI", IntakeConstants.pivotKI),
            pivotKD = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kD", IntakeConstants.pivotKD),
            pivotGravityArmPositionOffset = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/pivot/gravityArmPositionOffset",
                IntakeConstants.pivotGravityArmPositionOffset),
            pivotKG = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/kG", IntakeConstants.pivotKG),
            pivotMotionMagicCruiseVelocity = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/pivot/motionMagicCruiseVelocity",
                IntakeConstants.pivotMotionMagicCruiseVelocity),
            pivotMotionMagicAcceleration = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/pivot/motionMagicAcceleration",
                IntakeConstants.pivotMotionMagicAcceleration),
            pivotMotionMagicJerk = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/pivot/motionMagicJerk",
                IntakeConstants.pivotMotionMagicJerk),

        // non-motor-config constants
        pivotRunMotorAmps = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/runMotorAmps",
            IntakeConstants.runMotorAmps),
        // pivotHoldDownAmps = new LoggedNetworkNumber(IntakeConstants.dashboardPath +
        // "/pivotHoldDownAmps", IntakeConstants.pivotHoldDownAmps),
        pivotTorqueDownwardAmps = new LoggedNetworkNumber(
            IntakeConstants.dashboardPath + "/pivotTorqueDownwardAmps",
            IntakeConstants.pivotTorqueDownwardAmps),

        stallDebounceSeconds = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/stallDebounceSeconds",
            IntakeConstants.stallDebounceSeconds),
            stallCurrentAmpsThreshold = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/stallCurrentAmpsThreshold",
                IntakeConstants.stallCurrentAmpsThreshold),
            stallVelocityRpsThreshold = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/stallVelocityRpsThreshold",
                IntakeConstants.stallVelocityRpsThreshold),

        tuckVoltage = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/tuckVoltage",
            IntakeConstants.tuckVoltage),
            tuckHoldVoltage = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/tuckHoldVoltage",
                IntakeConstants.tuckHoldVoltage),
            extendVoltage = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/extendVoltage",
                IntakeConstants.extendVoltage),
            runMotorVoltage = new LoggedNetworkNumber(IntakeConstants.dashboardPath + "/runMotorVoltage",
                IntakeConstants.runMotorVoltage),

        ln_pivotPositionTolerance = new LoggedNetworkNumber(
            IntakeConstants.dashboardPath + "/pivotPositionTolerance",
            IntakeConstants.pivotPositionTolerance),
            ln_extendedEncoderPosition = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/extendedEncoderPosition",
                IntakeConstants.extendedEncoderPosition),
            ln_tuckedEncoderPosition = new LoggedNetworkNumber(
                IntakeConstants.dashboardPath + "/tuckedEncoderPosition",
                IntakeConstants.tuckedEncoderPosition);

        public void updateDashboardConfig() {
            if (!tuningEnabled.get())
                return;

            Slot0Configs slot = pivotMotorConfig.Slot0;
            MotionMagicConfigs mm = pivotMotorConfig.MotionMagic;

            boolean changed = false;

            double newKS = pivotKS.get();
            if (slot.kS != newKS) {
                slot.kS = newKS;
                changed = true;
                IntakeConstants.pivotKS = newKS;
            }

            double newKV = pivotKV.get();
            if (slot.kV != newKV) {
                slot.kV = newKV;
                changed = true;
                IntakeConstants.pivotKV = newKV;
            }

            double newKA = pivotKA.get();
            if (slot.kA != newKA) {
                slot.kA = newKA;
                changed = true;
                IntakeConstants.pivotKA = newKA;
            }

            double newKP = pivotKP.get();
            if (slot.kP != newKP) {
                slot.kP = newKP;
                changed = true;
                IntakeConstants.pivotKP = newKP;
            }

            double newKI = pivotKI.get();
            if (slot.kI != newKI) {
                slot.kI = newKI;
                changed = true;
                IntakeConstants.pivotKI = newKI;
            }

            double newKD = pivotKD.get();
            if (slot.kD != newKD) {
                slot.kD = newKD;
                changed = true;
                IntakeConstants.pivotKD = newKD;
            }

            double newGravityOffset = pivotGravityArmPositionOffset.get();
            if (slot.GravityArmPositionOffset != newGravityOffset) {
                slot.GravityArmPositionOffset = newGravityOffset;
                changed = true;
                IntakeConstants.pivotGravityArmPositionOffset = newGravityOffset;
            }

            double newKG = pivotKG.get();
            if (slot.kG != newKG) {
                slot.kG = newKG;
                changed = true;
                IntakeConstants.pivotKG = newKG;
            }

            double newCruise = pivotMotionMagicCruiseVelocity.get();
            if (mm.MotionMagicCruiseVelocity != newCruise) {
                mm.MotionMagicCruiseVelocity = newCruise;
                changed = true;
                IntakeConstants.pivotMotionMagicCruiseVelocity = newCruise;
            }

            double newAcc = pivotMotionMagicAcceleration.get();
            if (mm.MotionMagicAcceleration != newAcc) {
                mm.MotionMagicAcceleration = newAcc;
                changed = true;
                IntakeConstants.pivotMotionMagicAcceleration = newAcc;
            }

            double newJerk = pivotMotionMagicJerk.get();
            if (mm.MotionMagicJerk != newJerk) {
                mm.MotionMagicJerk = newJerk;
                changed = true;
                IntakeConstants.pivotMotionMagicJerk = newJerk;
            }

            // non-motor-config constants: update unconditionally
            IntakeConstants.runMotorAmps = pivotRunMotorAmps.get();
            // IntakeConstants.pivotHoldDownAmps = pivotHoldDownAmps.get();
            IntakeConstants.pivotTorqueDownwardAmps = pivotTorqueDownwardAmps.get();

            IntakeConstants.stallDebounceSeconds = stallDebounceSeconds.get();
            IntakeConstants.stallCurrentAmpsThreshold = stallCurrentAmpsThreshold.get();
            IntakeConstants.stallVelocityRpsThreshold = stallVelocityRpsThreshold.get();

            IntakeConstants.tuckVoltage = tuckVoltage.get();
            IntakeConstants.tuckHoldVoltage = tuckHoldVoltage.get();
            IntakeConstants.extendVoltage = extendVoltage.get();
            IntakeConstants.runMotorVoltage = runMotorVoltage.get();

            IntakeConstants.pivotPositionTolerance = ln_pivotPositionTolerance.get();
            IntakeConstants.extendedEncoderPosition = ln_extendedEncoderPosition.get();
            IntakeConstants.tuckedEncoderPosition = ln_tuckedEncoderPosition.get();
            if (changed) {

                // Applying the configs can be blocking/flash operation, so only do it when
                // necessary
                pivotMotorConfig = pivotMotorConfig
                    .withSlot0(slot)
                    .withMotionMagic(mm);
                pivotMotor.getConfigurator().apply(pivotMotorConfig);
            }
        }
    }
}
