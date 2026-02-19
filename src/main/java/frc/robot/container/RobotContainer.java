// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {

        public class Subsystems {
                public Intake intake;
                public Spindexer spindexer;
                public CommandSwerveDrivetrain drivetrain;
                public Controller controller;
        }

        Subsystems subsystems;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        public RobotContainer() {
                this.subsystems = new Subsystems();
                // subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                this.subsystems.controller = new Controller();
                subsystems.drivetrain = TunerConstants.createDrivetrain();
                this.subsystems.intake = new Intake();
                this.subsystems.spindexer = new Spindexer();

                this.subsystems.drivetrain.setDefaultCommand(this.subsystems.drivetrain.driveCommand());

                                this.subsystems.controller.getDriveController().y().onTrue(subsystems.intake.extendIntake());
                this.subsystems.controller.getDriveController().a().onTrue(subsystems.intake.tuck());
                this.subsystems.controller.getDriveController().x().onTrue(subsystems.intake.runIntake());
                this.subsystems.controller.getDriveController().b().onTrue(subsystems.intake.stopIntake());

                this.subsystems.controller.getDriveController().rightBumper().onTrue(this.subsystems.spindexer.spinKickCommand());
                this.subsystems.controller.getDriveController().leftBumper().onTrue(this.subsystems.spindexer.stopSpinKickCommand());

                Controller.zeroGyro.onTrue(this.subsystems.drivetrain.runOnce(
                                () -> this.subsystems.drivetrain.seedFieldCentric(Rotation2d.kZero)));
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return null;
        }

        public void teleopInit() {
                this.subsystems.intake.zeroPivot();
                // Any teleop-specific initialization code can go here.
        }

        public void teleopPeriodic() {

        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }
}
