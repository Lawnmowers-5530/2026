package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degree;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Bindings;


public class Intake extends SubsystemBase {
    public enum States {
        EXTENDED, TUCKED, TUCKING, EXTENDING
    }

    private States currentState = States.TUCKED;
    
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
        return Commands.either(Commands.none(), 
        Commands.runOnce(() -> {
            setState(States.TUCKING);
            runMotor.set(0);
            pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.TUCKED_ENCODER_POSITION));
        }, this)
        .andThen(Commands.waitUntil(this::pivotAtTuckPosition))
        .andThen(Commands.runOnce(() -> setState(States.TUCKED), this))
        , () -> {return !canTuck();});
    }
    public Command runIntake() {
        return Commands.either(Commands.runOnce(()->runMotor.setControl(new TorqueCurrentFOC(IntakeConstants.RUN_MOTOR_AMPS))), Commands.runOnce(()->runMotor.set(0)), this::canRunIntake);
    }
    public Command stopIntake() {
        return Commands.runOnce(()->runMotor.setControl(new TorqueCurrentFOC(0)));
    }
    public Command extendIntake() {
        return Commands.either(Commands.runOnce(()-> {
            setState(States.EXTENDING);
            pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.EXTENDED_ENCODER_POSITION));
        }, this)
        .andThen(Commands.waitUntil(this::pivotAtExtensionPosition))
        .andThen(Commands.runOnce(()-> {
            setState(States.EXTENDED);
            pivotMotor.setControl(new TorqueCurrentFOC(IntakeConstants.PIVOT_HOLD_DOWN_AMPS));
        }, this))
        , Commands.none(), this::canExtend);
    }
    private boolean canRunIntake() {
        return currentState == States.EXTENDED;
    }
    private boolean canExtend() {
        return currentState == States.TUCKED || currentState == States.TUCKING;
    }
    private boolean canTuck() {
        return currentState == States.EXTENDED || currentState == States.EXTENDING;
    }
    
    private void setState(States state) {
        currentState = state;
    }
    public States getState() {
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
