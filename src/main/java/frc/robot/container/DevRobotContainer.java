// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.Intake;

public class DevRobotContainer {

        public class Subsystems {
                public Intake intake;
                public Spindexer spindexer;
                public CommandSwerveDrivetrain drivetrain;
                public Turret turret;
        }
        Supplier<LinearVelocity> robotVelocitySupplier;

        Subsystems subsystems;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        private double pitchsp;
        private double yawsp;

        public DevRobotContainer() {

                this.pitchsp = 0;
                this.yawsp = 0;
                this.subsystems = new Subsystems();
               
                // subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                subsystems.drivetrain = TunerConstants.createDrivetrain();
                robotVelocitySupplier = new Supplier<LinearVelocity>() {
                        @Override
                        public LinearVelocity get() {
                                return LinearVelocity.ofBaseUnits(-subsystems.drivetrain.getState().Speeds.vxMetersPerSecond, MetersPerSecond);
                        }
                 };
                this.subsystems.intake = new Intake(robotVelocitySupplier);
                
                this.subsystems.spindexer = new Spindexer();
                this.subsystems.turret = new Turret();

                this.subsystems.drivetrain.setDefaultCommand(this.subsystems.drivetrain.driveCommand());
                Controller.getInstance().getDriveController().y().onTrue(subsystems.intake.applyTorqueDownward());
                Controller.getInstance().getDriveController().a().onTrue(subsystems.intake.tuck());
                Controller.getInstance().getDriveController().x().onTrue(subsystems.intake.runIntake());
                Controller.getInstance().getDriveController().b().onTrue(subsystems.intake.stopIntake());

                Controller.getInstance().getDriveController().rightBumper()
                                .onTrue(this.subsystems.spindexer.spinKickCommand());
                Controller.getInstance().getDriveController().leftBumper()
                                .onTrue(this.subsystems.spindexer.stopCommand());
                Controller.getInstance().getDriveController().povUp()
                                .onTrue(this.subsystems.turret.setFlywheelSpeedCommand(0));
                Controller.getInstance().getDriveController().povRight()
                                .onTrue(this.subsystems.turret.setFlywheelSpeedCommand(60));

                Controller.getInstance().getDriveController().povDown().onTrue(new InstantCommand(() -> {
                        this.subsystems.drivetrain.resetPose(new Pose2d());
                }, this.subsystems.drivetrain));

                Command path;
                PathConstraints constraints = new PathConstraints(3, 2, 2, 2);
                try {
                        path = AutoBuilder.followPath(PathPlannerPath.fromPathFile("testpath2"));
                } catch (Exception e) {
                        path = Commands.none();
                }
                SmartDashboard.putBoolean("isConfiguredPath", AutoBuilder.isPathfindingConfigured());
                SmartDashboard.putBoolean("isConfigured", AutoBuilder.isConfigured());
                Command a = AutoBuilder.pathfindToPose(new Pose2d(2, 2, new Rotation2d()), constraints);
                Controller.getInstance().getSecondaryController().povUp().onTrue(path);

                Controller.getInstance().zeroGyro.onTrue(this.subsystems.drivetrain.runOnce(
                                () -> this.subsystems.drivetrain.seedFieldCentric(Rotation2d.kZero)));

                Controller.getInstance().getSecondaryController().x().onTrue(new InstantCommand(() -> {this.subsystems.turret.zeroYaw();}, this.subsystems.turret));

                 
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return null;
        }

        public void teleopInit() {
                this.subsystems.intake.zeroPivot();
                this.subsystems.turret.zeroPitch();
                SmartDashboard.putNumber("shooter speed", 0);
                // Any teleop-specific initialization code can go here.
        }

        public void teleopPeriodic() {
                this.pitchsp += MathUtil.applyDeadband(Controller.getInstance().getSecondaryController().getLeftY(),
                                0.08) * 5;
                this.subsystems.turret.setPitch(Rotation2d.fromDegrees(this.pitchsp));

                this.yawsp += MathUtil.applyDeadband(Controller.getInstance().getSecondaryController().getRightX(),
                                0.08) * 0.5;
                
                this.subsystems.turret.setFlywheelSpeed(SmartDashboard.getNumber("shooter speed", 0));
                
                SmartDashboard.putString("pose", this.subsystems.drivetrain.getState().Pose.getTranslation().toString());
                //this.subsystems.turret.setYaw(angle);
                Rotation2d angle = LauncherConstants.blueTargetPose.minus(this.subsystems.drivetrain.getState().Pose.getTranslation().plus((LauncherConstants.distFromCenter.rotateBy(this.subsystems.drivetrain.getState().Pose.getRotation())))).getAngle().minus(this.subsystems.drivetrain.getState().Pose.getRotation());
                this.subsystems.turret.setYaw(angle);

        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }

        private Rotation2d getPitchSp() {
                return Rotation2d.fromDegrees(this.pitchsp);
        }
}
