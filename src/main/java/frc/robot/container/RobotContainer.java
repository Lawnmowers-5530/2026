// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.lib.BuildMetadata;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Bindings;
import frc.robot.Telemetry;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Indexer.Indexer;

public class RobotContainer {

        public class Subsystems {
                public CommandSwerveDrivetrain drivetrain;
        }

        Subsystems subsystems;
        Bindings bindings;

        private final SendableChooser<Command> autoChooser;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        public RobotContainer() {
                subsystems = new Subsystems();
                this.subsystems.drivetrain = TunerConstants.createDrivetrain();

                autoChooser = AutoBuilder.buildAutoChooser("Tests");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();

                // #region Smart Dashboard Programming Related
                if (_programmingDashboard) {
                        SmartDashboard.putData(metadata);
                }

                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        }

        private void configureBindings() {

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                this.subsystems.drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                this.subsystems.drivetrain.registerTelemetry(logger::telemeterize);

        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
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
