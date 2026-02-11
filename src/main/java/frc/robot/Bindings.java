package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Controller;

public final class Bindings {

    Robot.RobotContainer.Subsystems subsystems;
    Controller controller = new Controller();

    public Bindings(Robot.RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        drivetrain = this.new Drivetrain();

        Controller.zeroGyro.onTrue(this.drivetrain.zeroGyro());
    }

    Drivetrain drivetrain;

    final class Drivetrain {
        Command zeroGyro() {
            return new InstantCommand(() -> {
                subsystems.drivetrain.seedFieldCentric();
            }, subsystems.drivetrain);
        }
    }
}
