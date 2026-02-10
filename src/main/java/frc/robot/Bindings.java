package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Bindings {
    
    public class IntakeBindings {
        public static Trigger runWheelMotor = new Trigger(() -> {
            return Robot.controller.getLeftTriggerAxis() > 0.5;
        });
        public static Trigger toggleIntakeExtension = new Trigger(Robot.controller::getLeftBumperButtonPressed);
    }
}
