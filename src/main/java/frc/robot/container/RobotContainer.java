// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {

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

        SendableChooser<Command> autoChooser;

        boolean _programmingDashboard = true;

        private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

        private double pitchSp;

        public RobotContainer() {
                this.subsystems = new Subsystems();

                // subsystems.launcherFlywheel = new LauncherFlywheel(LauncherConstants.canId);
                this.subsystems.drivetrain = TunerConstants.createDrivetrain();
                this.subsystems.intake = new Intake(robotVelocitySupplier);
                this.subsystems.spindexer = new Spindexer();
                this.subsystems.turret = new Turret();

                this.bindings = new Bindings(this.subsystems);

                this.subsystems.drivetrain.setDefaultCommand(this.bindings.drivetrain.drive());
                this.subsystems.turret.setDefaultCommand(this.bindings.turret.autoAim());

                autoChooser = AutoBuilder.buildAutoChooser("singleShot");
                SmartDashboard.putData("autoChooser", autoChooser);

                NamedCommands.registerCommand("spinKick", this.bindings.spindexer.spinKick());
                new EventTrigger("feed2s").onTrue(
                                new ParallelDeadlineGroup(new WaitCommand(2), this.bindings.spindexer.spinKick()));
                new EventTrigger("Extend And Run Intake").onTrue(this.bindings.intake.toggleCollect());
                SmartDashboard.putNumber("set turret velo", 0);
                SmartDashboard.putNumber("set dist to hub", 0);

                this.pitchSp = 65;
                Controller.getInstance().getDriveController().rightTrigger(0.3)
                                .toggleOnTrue(this.bindings.turret.autoPass());
                Controller.getInstance().getDriveController().leftBumper()
                                .toggleOnTrue(this.bindings.intake.toggleCollect());
                Controller.getInstance().getDriveController().rightBumper()
                                .toggleOnTrue(this.bindings.spindexer.spinKick());
                Controller.getInstance().getSecondaryController().x().onTrue(this.bindings.drivetrain.zeroGyro());

                Controller.getInstance().getSecondaryController().rightBumper().toggleOnTrue(this.bindings.spindexer.spinKick().alongWith(this.bindings.turret.autoAim()));
                Controller.getInstance().getSecondaryController().leftBumper().toggleOnTrue(this.bindings.spindexer.passSpinKick().alongWith(this.bindings.turret.autoPass()));

                // Controller.getInstance().getDriveController().y().toggleOnTrue(bindings.turret.turretState1());
                // Controller.getInstance().getDriveController().a().toggleOnTrue(bindings.turret.turretState2());

                Controller.getInstance().getSwitches().b().whileTrue(subsystems.intake
                                .manualIntakeControl(Controller.getInstance().secondaryTriggerAxesSum));
                Controller.getInstance().getSwitches().x().onTrue(new InstantCommand(() -> {
                        this.subsystems.drivetrain.resetPose(new Pose2d());
                }));

                // Controller.getInstance().getSecondaryController().y().toggleOnTrue(this.subsystems.intake.manualPivotControl(()
                // -> {return Controller.getInstance().getSecondaryController().getLeftY();}));
                Controller.getInstance().getSecondaryController().y()
                                .onTrue(this.subsystems.intake.extendIntakeCommand());
                Controller.getInstance().getSecondaryController().a()
                                .onTrue(this.subsystems.intake.tuckIntakeCommand());

                //
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return autoChooser.getSelected();
        }

        public void robotInit() {
                this.subsystems.drivetrain.resetPose(new Pose2d(0.482, 7.58, Rotation2d.kZero));
                // this.subsystems.turret.zeroYaw();
        }

        public void teleopInit() {
                this.subsystems.intake.zeroPivot();
                this.subsystems.turret.zeroYaw();
                this.subsystems.turret.zeroPitch();
                SmartDashboard.putNumber("turret velo", 0);
                this.subsystems.turret.setPitch(Rotation2d.fromDegrees(pitchSp));
        }

        public void teleopPeriodic() {
                // SmartDashboard.putString("pose",
                // this.subsystems.drivetrain.getState().Pose.getTranslation().toString());
                pitchSp += MathUtil.applyDeadband(Controller.getInstance().getSecondaryController().getLeftY(), 0.07);
                // SmartDashboard.putNumber("pitchSp", pitchSp);

                SmartDashboard.putString(
                                "pose", this.subsystems.drivetrain.getState().Pose.toString());
                // this.subsystems.turret.setYaw(Rotation2d.kZero);
                // this.subsystems.turret.setPitch(Rotation2d.fromDegrees(pitchSp));
                // this.subsystems.turret.setFlywheelSpeed(SmartDashboard.getNumber("turret
                // velo", 0));
                // SmartDashboard.putString("turretState",
                // this.subsystems.turret.getTurretState().toString());
                // Rotation2d angle =
                // LauncherConstants.blueTargetPose.minus(this.subsystems.drivetrain.getState().Pose.getTranslation().plus((LauncherConstants.distFromCenter.rotateBy(this.subsystems.drivetrain.getState().Pose.getRotation())))).getAngle().minus(this.subsystems.drivetrain.getState().Pose.getRotation());

        }

        public void teleopExit() {
                // Any teleop-specific cleanup code can go here.
        }
}
