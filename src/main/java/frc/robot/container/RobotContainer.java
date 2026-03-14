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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.BuildMetadata;
import frc.robot.Telemetry;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {

    private final Telemetry logger = new Telemetry(SwerveConstants.maxSpeed);
    Supplier<LinearVelocity> robotVelocitySupplier;

    Subsystems subsystems;
    Bindings bindings;

    BuildMetadata metadata = new BuildMetadata();

    SendableChooser<Command> autoChooser;

    boolean _programmingDashboard = true;
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
        this.subsystems.turret.setDefaultCommand(this.subsystems.turret.smartDashboardTurretCommand("Turret Pitch", "Turret Speed", "Turret Yaw"));

        autoChooser = AutoBuilder.buildAutoChooser("singleShot");
        SmartDashboard.putData("autoChooser", autoChooser);

        NamedCommands.registerCommand("spinKick", this.bindings.spindexer.spinKick());
        new EventTrigger("feed2s").onTrue(
                new ParallelDeadlineGroup(new WaitCommand(2), this.bindings.spindexer.spinKick()));
        new EventTrigger("Extend And Run Intake").onTrue(this.subsystems.intake.toggleIntakeExtensionCommand().alongWith(this.subsystems.intake.runIntakeCommand()));
       
    
        // Controller.getInstance().getDriveController().y().toggleOnTrue(bindings.turret.turretState1());
        // Controller.getInstance().getDriveController().a().toggleOnTrue(bindings.turret.turretState2());
        Controller.getInstance().getDriveController().x().onTrue(this.bindings.drivetrain.zeroGyro());

        //TODO: THis needs to 1) Slow down drivetrain, 2) figure out when to spin indexer, 3) do auto aim
        Controller.getInstance().getDriveController().rightTrigger(0.3).whileTrue(this.bindings.turret.smartShootingCommand()); 
    
        Controller.getInstance().getSecondaryController().rightBumper().onTrue(this.subsystems.intake.toggleIntakeExtensionCommand());
        Controller.getInstance().getSecondaryController().leftBumper().whileTrue(this.subsystems.intake.jiggleIntakeCommand());
        Controller.getInstance().getSecondaryController().b().onTrue(this.subsystems.intake.runIntakeCommand());
        Controller.getInstance().getSecondaryController().a().onTrue(this.subsystems.intake.stopIntakeCommand());

        Controller.getInstance().getSwitches().b().whileTrue(subsystems.intake
                .manualIntakeControl(Controller.getInstance().secondaryTriggerAxesSum));
        Controller.getInstance().getSwitches().x().onTrue(new InstantCommand(() -> {
            this.subsystems.drivetrain.resetPose(new Pose2d());
        }));

        // Controller.getInstance().getSecondaryController().y().toggleOnTrue(this.subsystems.intake.manualPivotControl(()
        // -> {return Controller.getInstance().getSecondaryController().getLeftY();}));
       
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void robotInit() {
        this.subsystems.zero();
    }
 
    public void teleopInit() {
       
    }

    public void teleopPeriodic() {
        // SmartDashboard.putString("pose",
        // this.subsystems.drivetrain.getState().Pose.getTranslation().toString());
        // SmartDashboard.putNumber("pitchSp", pitchSp);
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

    public class Subsystems {
        public Intake intake;
        public Spindexer spindexer;
        public CommandSwerveDrivetrain drivetrain;
        public Turret turret;
        public void zero() {
            this.drivetrain.resetPose(new Pose2d(0.482, 7.58, Rotation2d.kZero));
            this.intake.zeroPivot();
            this.turret.zeroYaw();
            this.turret.zeroPitch();
        }
    }
}
