package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;

public final class Bindings {
    
    public class IntakeBindings {
        public static BooleanSupplier runWheelMotor = () -> {
            return Robot.controller.getLeftTriggerAxis() > 0.5;
        };
        public static BooleanSupplier toggleIntakeExtension = Robot.controller::getLeftBumperButtonPressed;
    }
}
