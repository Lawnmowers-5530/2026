package frc.robot.constants;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import lombok.experimental.UtilityClass;

@UtilityClass
public class VisionConstants {
    public final String kCameraName = "main";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public final Transform3d kRobotToCamBack =
            new Transform3d(new Translation3d(Units.inchesToMeters(-27.5 / 2), Units.inchesToMeters(27 / 2 - 8), Units.inchesToMeters(7.5)), new Rotation3d(Math.toRadians(5), Math.toRadians(45), Math.toRadians(180)));
    public final Transform3d kRobotToCamLeft = //TODO
            new Transform3d(new Translation3d(Units.inchesToMeters(27.5 / 2 - 3), Units.inchesToMeters(26.5 / 2), Units.inchesToMeters(8.5)), new Rotation3d(Math.toRadians(16.5), Math.toRadians(30), Math.toRadians(0)));
    // The layout of the AprilTags on the field
    public final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
