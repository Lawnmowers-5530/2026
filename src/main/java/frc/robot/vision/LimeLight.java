package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.RawDetection;


public class LimeLight extends SubsystemBase{

    List<Translation2d> ballPoses;
    //Constant
    private static final double kProjectionConstant = 1;
    private static final double kHorizontalMaxAngleDegrees = 70; 
    private static final double kVerticalMaxAngleDegrees = 50;
    private static Translation2d kLimelightResolution;
    private static Translation3d robotToLimelightTranslation;
    private static Rotation3d cameraRotationOnRobot;
    static {
        
        kLimelightResolution = new Translation2d(800,640);


    }

    public LimeLight() {
        ballPoses = new ArrayList<>();
    }

    @Override
    public void periodic() {
        ballPoses.clear();
        RawDetection[] ballsWdKnow = LimelightHelpers.getRawDetections("");
        for (RawDetection detection : ballsWdKnow) {
            ballPoses.add(calculateBallPoseFromData(detection));
        }
    }
    public List<Translation2d> getBallPositions() {
        return ballPoses;
    }
    private Translation2d calculateBallPoseFromData(RawDetection data) {
        Translation3d ballToCamera = getBallToCamera(data);
        Translation3d ballPosition3d = ballToCamera.rotateBy(cameraRotationOnRobot).plus(robotToLimelightTranslation);
        //Transform ball to camera into a translat
        return new Translation2d(ballPosition3d.getX(), ballPosition3d.getY());
    }
    private Translation3d getBallToCamera(RawDetection data) {
        double ballDiamterInPixels = Math.abs(data.corner0_X- data.corner1_X);
        double distance = kProjectionConstant/ballDiamterInPixels;
        SmartDashboard.putNumber("Distance", distance);
        double xAngle = data.txnc;
        double yAngle = data.tync;
        return new Translation3d(distance, new Rotation3d(0, Math.toRadians(yAngle), Math.toRadians(xAngle)));
    }
  

}
