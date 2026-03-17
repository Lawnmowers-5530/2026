package frc.robot.container;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Turret.TurretState;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.SwerveConstants;

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
                    .withDeadband(SwerveConstants.maxSpeed * 0.1)
                    .withRotationalDeadband(SwerveConstants.maxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            return subsystems.drivetrain.applyRequest(
                    () -> drive
                            .withVelocityX(Controller.getInstance().driveVector.get().get(0) * SwerveConstants.maxSpeed) // Drive
                            // forward
                            // with
                            // negative Y (forward)
                            .withVelocityY(Controller.getInstance().driveVector.get().get(1) * SwerveConstants.maxSpeed) // Drive
                            // left
                            // with
                            // negative X (left)
                            .withRotationalRate(Controller.getInstance().driveRotation.get() * SwerveConstants.maxAngularRate) // Drive
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

    }

    final class Turret {
        public static boolean getDrivetrainInAllianceZone(Alliance alliance, Pose2d pose) {
            return switch (alliance) {
                case Red -> pose.getMeasureX().in(Meters) > 12.5;
                case Blue -> pose.getMeasureY().in(Meters) < 4;
            };
        }

        //Command autoAim() {
        //    Translation3d target = DriverStation.getAlliance().isPresent()
        //            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? LauncherConstants.redTargetPose
        //                    : LauncherConstants.blueTargetPose;
//
        //    return subsystems.turret.setTurretStateCommand(() -> {
        //        Rotation2d rot = subsystems.drivetrain.getState().Pose.getRotation();
        //        Translation2d relTurretPos = LauncherConstants.distFromCenter.rotateBy(rot);
        //        Rotation2d theta = relTurretPos.getAngle().plus(Rotation2d.fromDegrees(90));
        //        double r = subsystems.drivetrain.getState().Speeds.omegaRadiansPerSecond
        //                * LauncherConstants.distFromCenter.getNorm();
        //        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds
        //                .fromRobotRelativeSpeeds(subsystems.drivetrain.getState().Speeds, rot);
        //        Vector<N2> turretVel = VecBuilder.fill(r * theta.getCos() + fieldRelativeSpeeds.vxMetersPerSecond,
        //                r * theta.getSin() + fieldRelativeSpeeds.vyMetersPerSecond);
        //        Vector<N2> turretPos = subsystems.drivetrain.getState().Pose.getTranslation()
        //                .plus(LauncherConstants.distFromCenter.rotateBy(rot)).toVector()
        //                .plus(turretVel.times(LauncherConstants.feedTime));
        //        return ProjectileAimer.parabolicTurretState(target,
        //                turretPos,
        //                turretVel,
        //                -2)
        //                .rotateBy(rot.times(-1)); // Add in robot rotation
        //    });
        Command autoAim() {
            return subsystems.turret.setTurretStateCommand(() -> {
                if (DriverStation.getAlliance().isEmpty()) {
                    return new TurretState(TurretConstants.turretOffset, TurretConstants.pitchZeroAngle, 0);
                }
                Translation2d target = DriverStation.getAlliance().get() == Alliance.Red ? FlippingUtil.flipFieldPosition(TurretConstants.blueTargetPose.toTranslation2d()) : TurretConstants.blueTargetPose.toTranslation2d();
                Pose2d pose = subsystems.drivetrain.getState().Pose;
                Translation2d turretTranslation = pose.getTranslation().plus(TurretConstants.distFromCenter.rotateBy(pose.getRotation()));
                Translation2d turretVelo = new Translation2d(subsystems.drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond, subsystems.drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond);
                Translation2d robotVelo = new Translation2d(subsystems.drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond, subsystems.drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond);
                double rw = subsystems.drivetrain.getState().Speeds.omegaRadiansPerSecond * TurretConstants.distFromCenter.getNorm();
                Rotation2d theta = turretTranslation.getAngle().plus(Rotation2d.kCCW_90deg);
                Translation2d turretVeloRot = new Translation2d(rw * theta.getCos(), rw * theta.getSin());
                double dist = target.getDistance(turretTranslation);//SmartDashboard.getNumber("set dist to hub", 0);//pose.getTranslation().getDistance(LauncherConstants.blueTargetPose.toTranslation2d());
                Translation2d adjustedTargetPose = target.minus((turretVelo.plus(turretVeloRot)).times(TurretConstants.distToTOF.get(dist)));

                //  for (int i = 0; i < 2)

                Rotation2d pitchAngle = TurretConstants.launchHoodAngleMap.get(adjustedTargetPose.getDistance(turretTranslation));
                Rotation2d yaw = (adjustedTargetPose.minus(turretTranslation)).getAngle();
                double velo = TurretConstants.distToSpinrate.get(adjustedTargetPose.getDistance(turretTranslation));//SmartDashboard.getNumber("set turret velo", 0);
                TurretState state = new TurretState(yaw, pitchAngle, velo);
                SmartDashboard.putNumber("demanded yaw", yaw.getDegrees());
                return state.rotateBy(pose.getRotation().times(-1));
            });
        }

        Command smartShootingCommand() {
            return Commands.either(autoAim(), autoPass(), this::inAllianceZone);
        }

        Command autoPass() {
            return subsystems.turret.setTurretStateCommand(() -> {
                if (DriverStation.getAlliance().isEmpty()) {
                    return new TurretState(TurretConstants.turretOffset, TurretConstants.pitchZeroAngle, 0);
                }
                Pose2d pose = subsystems.drivetrain.getState().Pose;
                Translation2d target = getTargetFromCurrentPose(pose, DriverStation.getAlliance().get());
                Translation2d turretTranslation = pose.getTranslation().plus(TurretConstants.distFromCenter.rotateBy(pose.getRotation()));
                Translation2d turretVelo = new Translation2d(subsystems.drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond, subsystems.drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond);
                double dist = target.getDistance(turretTranslation);//SmartDashboard.getNumber("set dist to hub", 0);//pose.getTranslation().getDistance(LauncherConstants.bluePassingPose.toTranslation2d());
                Translation2d adjustedTargetPose = target.minus(turretVelo.times(TurretConstants.distToTOF.get(dist)));
                Rotation2d pitchAngle = TurretConstants.launchHoodAngleMapPassing.get(adjustedTargetPose.getDistance(turretTranslation));
                Rotation2d yaw = (adjustedTargetPose.minus(turretTranslation)).getAngle();
                double velo = TurretConstants.distToSpinratePassing.get(adjustedTargetPose.getDistance(turretTranslation));//SmartDashboard.getNumber("set turret velo", 0);
                TurretState state = new TurretState(yaw, pitchAngle, velo);
                SmartDashboard.putNumber("demanded yaw", yaw.getDegrees());
                return state.rotateBy(pose.getRotation().times(-1));
            });
        }

        Translation2d getTargetFromCurrentPose(Pose2d pose, Alliance alliance) {
            Translation2d passingPoseRed;
            if (pose.getY() > 4) {
                passingPoseRed = new Translation2d(14, MathUtil.clamp(pose.getX(), 6, 7.5));
            } else {
                passingPoseRed = new Translation2d(14, MathUtil.clamp(pose.getY(), 0.5, 2));
            }
            return alliance == Alliance.Blue ? FlippingUtil.flipFieldPosition(passingPoseRed) : passingPoseRed;
        }

        boolean inAllianceZone() {
            if (DriverStation.getAlliance().isEmpty()) {
                return false;
            }
            return getDrivetrainInAllianceZone(DriverStation.getAlliance().get(), subsystems.drivetrain.getState().Pose) ;
        }

        Command turretState1() {
            return new RunCommand(() -> {
                subsystems.turret.setTurretState(TurretConstants.state1);
            }, subsystems.turret);
        }

        Command turretState2() {
            return new RunCommand(() -> {
                subsystems.turret.setTurretState(TurretConstants.state2);
            }, subsystems.turret);
        }
    }

    final class Spindexer {
        Command spinKick() {
            return subsystems.spindexer.spinKickCommand().finallyDo(() -> {
                subsystems.spindexer.stopSpinKick();
            });
        }

        Command passSpinKick() {
            return subsystems.spindexer.spinKickFastCommand().finallyDo(() -> {
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
