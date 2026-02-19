package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ControllerConstants;

public class Controller {
    public CommandXboxController driverController;
    public CommandXboxController secondaryController;
    public CommandXboxController switches;

    public static Supplier<Vector<N2>> driveVector;
    public static Supplier<Double> driveRotation;
    public static Trigger slowMode;
    public static Trigger zeroGyro;

    public Controller() {
        this.driverController = new CommandXboxController(0);
        
        driveVector = () -> VecBuilder.fill(-this.driverController.getLeftY(), -this.driverController.getLeftX());
        driveRotation = () -> -this.driverController.getRightX();
    

    // driver controller 
    
        driveVector = () -> {
            return VecBuilder.fill(
                    MathUtil.applyDeadband(
                            -driverController.getLeftY(),
                            ControllerConstants.driveControllerJoystickDeadband,
                            1),
                    MathUtil.applyDeadband(
                            -driverController.getLeftX(),
                            ControllerConstants.driveControllerJoystickDeadband,
                            1));
        };

        driveRotation = () -> {
            return MathUtil.applyDeadband(
                    -driverController.getRightX() * 1.5,
                    ControllerConstants.driveControllerJoystickDeadband,
                    1);
        };
        slowMode = driverController.b();
        zeroGyro = driverController.povLeft();
    }
    

    public CommandXboxController getDriveController() {
        return this.driverController;
    }
}