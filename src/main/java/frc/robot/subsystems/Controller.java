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

    public Supplier<Vector<N2>> driveVector;
    public Supplier<Double> driveRotation;
    public Trigger slowMode;
    public Trigger zeroGyro;
    public Trigger runIntake;
    public Trigger reverseIntake;
    public Trigger extendIntake;
    public Trigger tuckIntake;

    private static Controller instance;

    public static Controller getInstance() {
        if (instance == null) {
            instance = new Controller();
        }
        return instance;
    }

    private Controller() {
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