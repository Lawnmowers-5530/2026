package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.RawDetection;


public class LimeLight {

 
    //Constant
    private static final double kProjectionConstant = 3200; //aproximate better lol
    private static final double kHorizontalMaxAngleDegrees = 70; 
    private static final double kVerticalMaxAngleDegrees = 50;
    //TODO: Add these constants in for limelight
    private static Translation3d robotToLimelightTranslation;
    private static Rotation3d cameraRotationOnRobot;
    static {
        
       


    }

    public LimeLight() {
       
    }
    
    public static List<Translation2d> getBallPositions() {
        //Clears the list everytime it is called so be careful
        List<Translation2d> ballPoses = new ArrayList<>();
        RawDetection[] ballsWdKnow = LimelightHelpers.getRawDetections("");

        for (RawDetection detection : ballsWdKnow) {
            ballPoses.add(calculateBallPoseFromData(detection));
        }
        return ballPoses;
    }
    private static Translation2d calculateBallPoseFromData(RawDetection data) {
        Translation3d ballToCamera = getBallToCamera(data);
        Translation3d ballPosition3d = ballToCamera.rotateBy(cameraRotationOnRobot).plus(robotToLimelightTranslation);
        //Transform ball to camera into a translat
        return new Translation2d(ballPosition3d.getX(), ballPosition3d.getY());
    }
    private static Translation3d getBallToCamera(RawDetection data) {
        //Uses pinhole model. If this fails, we need to adapt a thin lens model
        double ballDiamterInPixels = Math.max(Math.abs(data.corner0_X- data.corner2_X), Math.abs(data.corner0_Y - data.corner2_Y));
        //TODO: Change this to an interpolating tree map
        double distance = kProjectionConstant/ballDiamterInPixels;
        SmartDashboard.putNumber("Distance", distance);
        double xAngle = data.txnc;
        double yAngle = data.tync;
        return new Translation3d(distance, new Rotation3d(0, Math.toRadians(yAngle), Math.toRadians(xAngle)));
    }
  

}
