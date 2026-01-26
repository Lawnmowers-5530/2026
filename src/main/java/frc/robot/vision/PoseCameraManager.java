package frc.robot.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Manages all pose cameras. Designed to declutter
 * {@link Swerve Swerve}.
 */
public class PoseCameraManager {
    static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public ArrayList<PoseCamera> camList = new ArrayList<>();
    private CommandSwerveDrivetrain swerve;

    public PoseCameraManager(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
        camList.add(new PoseCamera("Front", new Transform3d()));
    }

    public static Pose3d getAprilTagPoseFromId(int id)  {
        return layout.getTagPose(id).get();
    }

    /**
     * Fetch estimated pose from all registered pose cameras paired with respective
     * std devs
     * 
     * @return List of Optional estimated robot poses from cameras paired with
     *         respective std devs
     */
    public ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getEstimatedPoses() {
        ArrayList<Pair<EstimatedRobotPose, Matrix<N3, N1>>> poseList = new ArrayList<>();
        Optional<EstimatedRobotPose> estimate;
        for (PoseCamera camera : camList) {
            estimate = camera.getPoseEstimate();
            estimate.ifPresent(estimatedRobotPose -> poseList.add(Pair.of(estimatedRobotPose,
                    camera.getEstimationStdDevs(estimatedRobotPose.estimatedPose.toPose2d()))));
        }
        return poseList;
    }

    public ArrayList<PhotonTrackedTarget> getTagsById(Optional<Integer> fiducialTagId) {
        ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();
        for (PoseCamera camera : camList) {
            camera.getTagById(fiducialTagId).ifPresent(targets::add);
        }
        return targets;
    }

    /**
     *
     * @param fiducialTagId Tag id to find yaw of
     * @return Double of yaw if tag id is primary tag in view of a registered camera
     */
    public Optional<Double> getFiducialIdYaw(int fiducialTagId) {
        for (PoseCamera camera : camList) {
            if (camera.getPrimaryTagId() == fiducialTagId) {
                return Optional.of(camera.getPrimaryTagYaw());
            }
        }
        return Optional.empty();
    }

    public void periodic() {

        var poseEstimates = this.getEstimatedPoses();

        for(var measurementPair : poseEstimates) {
            this.swerve.addVisionMeasurement(measurementPair);
        }

        
    }
}