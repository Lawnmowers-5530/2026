package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake extends SubsystemBase {
    public enum State {
        EXTENDED, TUCKED, TUCKING, EXTENDING
    }

    private State currentState = State.TUCKED;

    private TalonFX pivotMotor;
    private TalonFX runMotor;

    private static boolean exists = false;
    public static Intake instance;
    private Supplier<LinearVelocity> robotVelocity;
    private Debouncer stalliDebouncer;

    public Intake(Supplier<LinearVelocity> robotVelocity) {
        CommandScheduler.getInstance().registerSubsystem(this);
        if (Intake.exists) {
            System.err.println("Creating more than one intake. Please fix broken code");
        } else {
            stalliDebouncer = new Debouncer(0.05, DebounceType.kRising);
            this.robotVelocity = robotVelocity;
            instance = this;
            Intake.exists = true;
            pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_PORT, "canivore");
            runMotor = new TalonFX(IntakeConstants.RUN_MOTOR_PORT, "canivore");
            TalonFXConfiguration runMotorConfig = new TalonFXConfiguration();
            TorqueCurrentConfigs torqueCurrentConfig = new TorqueCurrentConfigs();
            runMotorConfig = runMotorConfig.withTorqueCurrent(torqueCurrentConfig);
            runMotor.getConfigurator().apply(runMotorConfig);
            TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
            var slot0PivotConfig = pivotMotorConfig.Slot0;
            slot0PivotConfig.kS = 0.25; // Add 0.25 V output to overcome static friction
            slot0PivotConfig.kV = 1; // A velocity target of 1 rps results in 0.12 V output
            slot0PivotConfig.kA = 1; // An acceleration of 1 rps/s requires 0.01 V output
            slot0PivotConfig.kP = 12; // A position error of 2.5 rotations results in 12 V output
            slot0PivotConfig.kI = 0; // no output for integrated error
            slot0PivotConfig.kD = 0; // A velocity error of 1 rps results in 0.1 V output
            slot0PivotConfig.GravityType = GravityTypeValue.Arm_Cosine;
            slot0PivotConfig.GravityArmPositionOffset = 0.1;
            slot0PivotConfig.kG = 0;

            var motionMagicConfigPivot = pivotMotorConfig.MotionMagic;
            motionMagicConfigPivot.MotionMagicCruiseVelocity = 1; // Target cruise velocity of 80 rps
            motionMagicConfigPivot.MotionMagicAcceleration = 1; // Target acceleration of 160 rps/s (0.5 seconds)
            motionMagicConfigPivot.MotionMagicJerk = 4; // Target jerk of 1600 rps/s/s (0.1 seconds)

            pivotMotor.getConfigurator().apply(pivotMotorConfig);

        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake State", this.currentState.toString());
        SmartDashboard.putString("Pivot pos", this.pivotMotor.getPosition().toString());
        SmartDashboard.putString("Target", this.pivotMotor.getAppliedControl().getControlInfo().toString());
        SmartDashboard.putBoolean("pivotAtExtensionPosition", this.pivotAtExtensionPosition());
        SmartDashboard.putNumber("applied output", this.pivotMotor.get());
        isStalling = stalliDebouncer.calculate(stallinBallin());
        // TODO Telemetry or logging
    }
    private boolean isStalling = false;
    public Command tuckIntakeCommand() {
        return Commands.runOnce(()->{
            this.currentState = State.TUCKING;
             runMotor.set(0);
            pivotMotor.setControl(new VoltageOut(-3));
        }, this).andThen(Commands.waitUntil(this::isStalling)).andThen(()->{this.currentState = State.TUCKED;pivotMotor.setControl(new VoltageOut(-0.4));}, this);
    }
    private boolean isStalling() {
        return isStalling;
    }
    public boolean stallinBallin() {
        return pivotMotor.getStatorCurrent().getValue().in(Amps) > 100 && Math.abs(pivotMotor.getVelocity().getValue().in(RotationsPerSecond)) < 5;
    }
    public void runIntake() {
        runMotor.setControl(new VoltageOut(7));
    }

    public void stopIntake() {
        runMotor.setControl(new TorqueCurrentFOC(0));
    }

    public Command runIntakeCommand() {
        return Commands.either(
                Commands.run(() -> {
                    SmartDashboard.putNumber("Intake speed", runMotor.get());
                    // runMotor.setControl(new TorqueCurrentFOC(IntakeConstants.RUN_MOTOR_AMPS));
                    this.runIntake();

                }, this),
                this.stopIntakeCommand(), () -> {
                    return true;
                });
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> {
            this.stopIntake();
        });
    }

    @Deprecated
    /**
     * DO NOT CALL THIS IN PRODUCTION CODE
     * 
     */
    public Command applyTorqueDownward() {
        return Commands.runOnce(() -> {
            pivotMotor.setControl(new TorqueCurrentFOC(15));
        }, this);
    }

    public Command extendIntakeCommand() {

        return Commands.either(new ParallelDeadlineGroup(Commands.waitUntil(this::isStalling),
                Commands.runOnce(() -> {
                    this.currentState = State.EXTENDING;
                    pivotMotor
                            .setControl(new VoltageOut(2));// new
                                                           // MotionMagicVoltage(IntakeConstants.EXTENDED_ENCODER_POSITION).withSlot(0));
                }, this))
                .andThen(Commands.runOnce(() -> {
                    this.currentState = State.EXTENDED;
                    pivotMotor.setControl(new TorqueCurrentFOC(IntakeConstants.PIVOT_HOLD_DOWN_AMPS));
                }, this)),
                new InstantCommand(), this::canExtend);
    }

    private boolean canRunIntake() {
        return currentState == State.EXTENDED;
    }

    private boolean canExtend() {
        return currentState == State.TUCKED || currentState == State.TUCKING;
    }

    private boolean canTuck() {
        return currentState == State.EXTENDED || currentState == State.EXTENDING;
    }

    public State getState() {
        return currentState;
    }

    private boolean pivotAtTuckPosition() {
        return Math.abs(pivotMotor.getPosition().getValueAsDouble() - IntakeConstants.TUCKED_ENCODER_POSITION) <= 0.1;
    }

    private boolean pivotAtExtensionPosition() {
        return Math.abs(pivotMotor.getPosition().getValueAsDouble() - IntakeConstants.EXTENDED_ENCODER_POSITION) <= 0.1;
    }

    public void zeroPivot() {
        this.pivotMotor.setPosition(0);
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

}
