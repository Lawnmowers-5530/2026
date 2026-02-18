package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degree;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    public enum State {
        EXTENDED, TUCKED, TUCKING, EXTENDING
    }

    private State currentState = State.TUCKED;

    private TalonFX pivotMotor;
    private TalonFX runMotor;


    private static boolean exists = false;
    public static Intake instance;

    public Intake() {
        if (Intake.exists) {
            System.err.println("Creating more than one intake. Please fix broken code");
        }else {
            instance = this;
            Intake.exists = true;
            pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_PORT);
            runMotor = new TalonFX(IntakeConstants.RUN_MOTOR_PORT);

        }

    }
    @Override
    public void periodic() {

        //TODO Telemetry or logging
    }

    public Command tuck() {
        return
        Commands.runOnce(() -> {
            if (!canTuck()) return;
            this.currentState = State.TUCKING;
            runMotor.set(0);
            pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.TUCKED_ENCODER_POSITION));
        }, this)
        .andThen(Commands.waitUntil(this::pivotAtTuckPosition))
        .andThen(Commands.runOnce(() -> this.currentState = State.TUCKED, this));
    }
    public Command runIntake() {
        return Commands.runOnce(()-> {
            if (canRunIntake()) {
                runMotor.setControl(new TorqueCurrentFOC(IntakeConstants.RUN_MOTOR_AMPS));
            } else {
                runMotor.setControl(new TorqueCurrentFOC(0));
            }
        }
        );
    }
    public Command stopIntake() {
        return Commands.runOnce(()->runMotor.setControl(new TorqueCurrentFOC(0)));
    }
    public Command extendIntake() {
        return Commands.either(Commands.runOnce(()-> {
            this.currentState = State.EXTENDING;
            pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.EXTENDED_ENCODER_POSITION));
        }, this)
        .andThen(Commands.waitUntil(this::pivotAtExtensionPosition))
        .andThen(Commands.runOnce(()-> {
            this.currentState = State.EXTENDED;
            pivotMotor.setControl(new TorqueCurrentFOC(IntakeConstants.PIVOT_HOLD_DOWN_AMPS));
        }, this))
        , Commands.none(), this::canExtend);
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
    //TODO: replace lt and gt with near for better results
    private boolean pivotAtTuckPosition() {
        return pivotMotor.getPosition().getValue().lt(Angle.ofBaseUnits(IntakeConstants.TUCKED_ENCODER_POSITION , Degree.getBaseUnit()));
    }
    private boolean pivotAtExtensionPosition() {
        return pivotMotor.getPosition().getValue().gt(Angle.ofBaseUnits(IntakeConstants.EXTENDED_ENCODER_POSITION , Degree.getBaseUnit()));
    }





}
