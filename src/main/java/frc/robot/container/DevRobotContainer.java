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
        Bindings bindings;

        BuildMetadata metadata = new BuildMetadata();

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        private double pitchSp;

        public DevRobotContainer() {
                this.subsystems = new Subsystems();
               
                // subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                this.subsystems.drivetrain = TunerConstants.createDrivetrain();
                this.subsystems.intake = new Intake(robotVelocitySupplier);
                this.subsystems.spindexer = new Spindexer();
                this.subsystems.turret = new Turret();

                this.bindings = new Bindings(this.subsystems);

                this.subsystems.drivetrain.setDefaultCommand(this.bindings.drivetrain.drive());
                this.subsystems.turret.setDefaultCommand(this.bindings.turret.autoAim());

                this.pitchSp = 0;

                Controller.getInstance().getDriveController().leftBumper().onTrue(this.bindings.intake.collect());
                Controller.getInstance().getDriveController().rightBumper().toggleOnTrue(this.subsystems.spindexer.spinKickCommand());
                Controller.getInstance().getDriveController().a().onTrue(this.bindings.drivetrain.zeroGyro());
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return null;
        }

        public void robotInit() {
                this.subsystems.drivetrain.resetPose(new Pose2d(0.417, 7.596, Rotation2d.kZero));
                //this.subsystems.turret.zeroYaw();
        }

        public void teleopInit() {
        }

        public void teleopPeriodic() {
                SmartDashboard.putString("pose", this.subsystems.drivetrain.getState().Pose.getTranslation().toString());
                pitchSp += MathUtil.applyDeadband(Controller.getInstance().getSecondaryController().getLeftY(), 0.07);
                SmartDashboard.putNumber("pitchSp", pitchSp);
                //this.subsystems.turret.setYaw(Rotation2d.fromDegrees(pitchSp));

                //Rotation2d angle = LauncherConstants.blueTargetPose.minus(this.subsystems.drivetrain.getState().Pose.getTranslation().plus((LauncherConstants.distFromCenter.rotateBy(this.subsystems.drivetrain.getState().Pose.getRotation())))).getAngle().minus(this.subsystems.drivetrain.getState().Pose.getRotation());

        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }
}
