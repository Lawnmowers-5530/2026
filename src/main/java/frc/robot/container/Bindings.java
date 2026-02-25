package frc.robot.container;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Bindings {

    RobotContainer.Subsystems subsystems;

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        drivetrain = this.new Drivetrain();

        Controller.getInstance().zeroGyro.onTrue(this.drivetrain.zeroGyro());
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
