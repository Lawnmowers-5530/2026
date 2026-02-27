package frc.robot.container;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Controller;
import frc.lib.ProjectileAimer;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class Bindings {

    DevRobotContainer.Subsystems subsystems;

    Drivetrain drivetrain;
    Intake intake;
    Turret turret;

    public Bindings(DevRobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        drivetrain = this.new Drivetrain();
        intake = this.new Intake();
        turret = this.new Turret();
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
        Command collect() {
            return subsystems.intake.extendIntake().andThen(subsystems.intake.runIntake());
        }
    }

    final class Turret {
        Command autoAim() {
            Translation3d target = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? LauncherConstants.redTargetPose
                            : LauncherConstants.blueTargetPose;

            return subsystems.turret.setTurretStateCommand(() -> {
                Rotation2d rot = subsystems.drivetrain.getState().Pose.getRotation();
                Translation2d turretPos = subsystems.drivetrain.getState().Pose.getTranslation().plus(LauncherConstants.distFromCenter.rotateBy(rot));
                Rotation2d theta = turretPos.getAngle().plus(Rotation2d.fromDegrees(90));
                double r = subsystems.drivetrain.getState().Speeds.omegaRadiansPerSecond * LauncherConstants.distFromCenter.getNorm();
                Vector<N2> turretVel = VecBuilder.fill(r * theta.getCos(), r * theta.getSin());
                return ProjectileAimer.parabolicTurretState(target,
                        turretPos,
                        turretVel,
                        -2)
                        .rotateBy(rot.times(-1)); //Add in robot rotation
            });
        };
    }
}
