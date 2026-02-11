package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Intake.Intake;

public final class Bindings {
    RobotContainer.Subsystems subsystems;
    Controller controller = new Controller();

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        drivetrain = this.new Drivetrain();

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
