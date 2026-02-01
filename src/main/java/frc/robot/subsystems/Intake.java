package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Bindings;


public class Intake {
    public enum States {
        EXTENDED, TUCKED, TUCKING, EXTENDING
    }

    private States currentState = States.TUCKED;
    
    private TalonFX pivotMotor;
    private TalonFX runMotor;

    private class IntakeConstants {
        //TODO find the actual values for these
        static final int PIVOT_MOTOR_PORT = 0;
        static final int RUN_MOTOR_PORT = 0;

        static final double RUN_MOTOR_AMPS = 20;


        static final double EXTENDED_ENCODER_POSITION = 0;
        static final double TUCKED_ENCODER_POSITION = 0;

        static final double PIVOT_HOLD_DOWN_AMPS = 0;
    }
    static boolean exists = false;
    
    public Intake() {
        if (Intake.exists) {
            System.err.println("Creating more than one intake. Please fix broken code");
        }else {
            Intake.exists = true;
            pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_PORT);
            runMotor = new TalonFX(IntakeConstants.RUN_MOTOR_PORT);
            
        }
        
    }
    public void periodic() {
        switch (currentState) {
            case EXTENDED: 
                pivotMotor.setControl(new TorqueCurrentFOC(IntakeConstants.PIVOT_HOLD_DOWN_AMPS));
                if (Bindings.IntakeBindings.runWheelMotor.getAsBoolean()) {
                    runMotor.setControl(new TorqueCurrentFOC(IntakeConstants.RUN_MOTOR_AMPS));
                }else {
                    runMotor.setControl(new TorqueCurrentFOC(0));
                }
                if (Bindings.IntakeBindings.toggleIntakeExtension.getAsBoolean()) {
                    currentState = States.TUCKING;
                    pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.TUCKED_ENCODER_POSITION));
                }
                
                break;
            case EXTENDING:
                if (pivotAtExtensionPosition()) {
                    currentState = States.EXTENDED;
                }
                break;
            case TUCKED:
                if (Bindings.IntakeBindings.toggleIntakeExtension.getAsBoolean()) {
                    pivotMotor.setControl(new MotionMagicExpoVoltage(IntakeConstants.EXTENDED_ENCODER_POSITION));
                    currentState = States.EXTENDING;
                }
                break;
            case TUCKING:
                if (pivotAtTuckPosition()) {
                    currentState = States.TUCKED;
                }
                break;
            default:
                break;
            
        }
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
