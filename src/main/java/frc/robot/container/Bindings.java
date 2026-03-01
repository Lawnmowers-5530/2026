package frc.robot.container;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Controller;
import frc.lib.ProjectileAimer;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Bindings {

    RobotContainer.Subsystems subsystems;

    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    Spindexer spindexer;

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        drivetrain = this.new Drivetrain();
        intake = this.new Intake();
        turret = this.new Turret();
        spindexer = this.new Spindexer();
    }

    final class Drivetrain {
        Command drive() {
            final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(SwerveConstants.MaxSpeed * 0.1)
                    .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            return subsystems.drivetrain.applyRequest(() -> drive
                    .withVelocityX(Controller.getInstance().driveVector.get().get(0) * SwerveConstants.MaxSpeed) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                    // negative Y (forward)
                    .withVelocityY(Controller.getInstance().driveVector.get().get(1) * SwerveConstants.MaxSpeed) // Drive
                                                                                                                 // left
                                                                                                                 // with
                    // negative X (left)
                    .withRotationalRate(Controller.getInstance().driveRotation.get() * SwerveConstants.MaxAngularRate) // Drive
            // counterclockwise
            // with
            // negative X (left)
            );
        }

        Command zeroGyro() {
            return new InstantCommand(() -> {
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                        subsystems.drivetrain.setOperatorPerspectiveForward(
                                subsystems.drivetrain.getState().Pose.getRotation().plus(Rotation2d.k180deg));
                    }
                }
                subsystems.drivetrain
                        .setOperatorPerspectiveForward(subsystems.drivetrain.getState().Pose.getRotation());
            }, subsystems.drivetrain);
        }

        Command resetPose() {
            return new InstantCommand(() -> {
                subsystems.drivetrain.resetPose(new Pose2d());
            }, subsystems.drivetrain);

        }
    }

    final class Intake {
        Command toggleCollect() {
            return subsystems.intake.extendIntakeCommand().andThen(subsystems.intake.runIntakeCommand()).finallyDo(() -> {CommandScheduler.getInstance().schedule(subsystems.intake.tuckIntakeCommand());});
        }

    }

    final class Turret {
        Command autoAim() {
            Translation3d target = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? LauncherConstants.redTargetPose
                            : LauncherConstants.blueTargetPose;

            return subsystems.turret.setTurretStateCommand(() -> {
                Rotation2d rot = subsystems.drivetrain.getState().Pose.getRotation();
                Translation2d relTurretPos = LauncherConstants.distFromCenter.rotateBy(rot);
                Rotation2d theta = relTurretPos.getAngle().plus(Rotation2d.fromDegrees(90));
                double r = subsystems.drivetrain.getState().Speeds.omegaRadiansPerSecond
                        * LauncherConstants.distFromCenter.getNorm();
                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds
                        .fromRobotRelativeSpeeds(subsystems.drivetrain.getState().Speeds, rot);
                Vector<N2> turretVel = VecBuilder.fill(r * theta.getCos() + fieldRelativeSpeeds.vxMetersPerSecond,
                        r * theta.getSin() + fieldRelativeSpeeds.vyMetersPerSecond);
                Vector<N2> turretPos = subsystems.drivetrain.getState().Pose.getTranslation()
                        .plus(LauncherConstants.distFromCenter.rotateBy(rot)).toVector()
                        .plus(turretVel.times(LauncherConstants.feedTime));
                return ProjectileAimer.parabolicTurretState(target,
                        turretPos,
                        turretVel,
                        -2)
                        .rotateBy(rot.times(-1)); // Add in robot rotation
            });
        };

        Command straightTurretAim() {
            return new RunCommand(() -> {
                subsystems.turret.setYaw(subsystems.drivetrain.getState().Pose.getRotation().times(-1));
            }, subsystems.turret);
        }

        Command turretState1() {
            return new RunCommand(() -> {subsystems.turret.setTurretState(LauncherConstants.state1);}, subsystems.turret);
        }
        Command turretState2() {
            return new RunCommand(() -> {subsystems.turret.setTurretState(LauncherConstants.state2);}, subsystems.turret);
        }
    }

    final class Spindexer {
        Command spinKick() {
            return subsystems.spindexer.spinKickCommand().finallyDo(() -> {
                subsystems.spindexer.stopSpinKick();
            });
        }

        Command reverse() {
            return subsystems.spindexer.reverseCommand().finallyDo(() -> {
                subsystems.spindexer.stopSpinKick();
            });
        }

        Command jitter() {
            return subsystems.spindexer.jitter().finallyDo(() -> {
                subsystems.spindexer.stopSpinKick();
            });
        }
    }
}
