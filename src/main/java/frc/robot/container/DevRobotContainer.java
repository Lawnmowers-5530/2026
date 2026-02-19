// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.vision.LimeLight;

public class DevRobotContainer {

        public class Subsystems {
                public CommandSwerveDrivetrain drivetrain;
        }

        Subsystems subsystems;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        private final SendableChooser<Command> autoChooser;

        public DevRobotContainer() {
                autoChooser = AutoBuilder.buildAutoChooser("TestAuto");

                LimeLight limelight = new LimeLight();
                this.subsystems = new Subsystems();
                // subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                subsystems.drivetrain = TunerConstants.createDrivetrain();

                this.subsystems.drivetrain.setDefaultCommand(this.subsystems.drivetrain.driveCommand());
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                final var idle = new SwerveRequest.Idle();
                PathConstraints constraints = new PathConstraints(SwerveConstants.MaxSpeed, SwerveConstants.MaxAcceleration, SwerveConstants.MaxAngularVelocity, SwerveConstants.MaxAngularAcceleration);
                AutoBuilder.pathfindToPose(new Pose2d(LimeLight.getBallPositions().get(0), this.subsystems.drivetrain.getRotation3d().toRotation2d()), constraints);
                return autoChooser.getSelected().andThen(
                                this.subsystems.drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        }

        public void teleopInit() {
                // Any teleop-specific initialization code can go here.
        }

        public void teleopPeriodic() {

        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }
}
