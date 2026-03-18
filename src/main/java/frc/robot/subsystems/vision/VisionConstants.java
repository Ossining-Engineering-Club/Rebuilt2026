package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static record PoseEstimate(
      Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> standardDev) {}

  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final String[] limelightNames = {"limelight-front", "limelight-left"};

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(3.0, 3.0, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final boolean IGNORE_YAW = false;

  // public static final double MAX_HEIGHT = 0.305;
  // public static final double MAX_ANGLE = 0.3;

  public static final double MAX_SINGLE_TAG_TRANSLATIONAL_DELTA = 1.5;

  public static final double SINGLE_TAG_AMBIGUITY_LIMIT = 0.7;

  // sim constants
  public static final Transform3d frontLLRobotToCamOffset =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(11.023221),
              Units.inchesToMeters(4.877645),
              Units.inchesToMeters(7.407200)),
          new Rotation3d(0, Units.degreesToRadians(-28.1), Units.degreesToRadians(10)));
  public static final Transform3d leftLLRobotToCamOffset =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(5.047364),
              Units.inchesToMeters(13.289059),
              Units.inchesToMeters(11.062867)),
          new Rotation3d(0, Units.degreesToRadians(-28.1), Units.degreesToRadians(84)));
  public static final double cameraDiagonalFOVDegrees = Math.hypot(82, 56.2);
}
