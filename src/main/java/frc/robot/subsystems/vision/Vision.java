package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.PoseEstimate;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] ios;
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... ios) {
    this.ios = ios;
    inputs = new VisionIOInputsAutoLogged[ios.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      // updating vision io inputs
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + limelightNames[i], inputs[i]);
    }
  }

  public PoseEstimate[] getEstimatedGlobalPoses(Pose2d robotPose, boolean isRobotOverBump) {
    List<PoseEstimate> estimates = new ArrayList<>();
    Set<Pose3d> detectedTagPoses = new HashSet<Pose3d>();
    for (int i = 0; i < ios.length; i++) {
      // adding detected tags to list to be logged
      for (int tagId : inputs[i].tagIds) {
        VisionConstants.TAG_LAYOUT
            .getTagPose(tagId)
            .ifPresent(
                (tagPose) ->
                    detectedTagPoses.add(tagPose)); // if there's a pose, add it to the list
      }

      boolean addedPose = false;
      if (inputs[i].estimateIsPresent) {
        // don't use if estimate is outside the field
        if (!(inputs[i].estimatedPose.getX() > 0.0
            && inputs[i].estimatedPose.getX() <= FieldConstants.fieldLengthMeters
            && inputs[i].estimatedPose.getY() > 0.0
            && inputs[i].estimatedPose.getY() <= FieldConstants.fieldWidthMeters)) continue;
        // don't use if estimate is too high, or too tilted
        // if (Math.abs(inputs[i].estimatedPose.getZ()) > VisionConstants.MAX_HEIGHT) continue;
        // if (Math.abs(inputs[i].estimatedPose.getRotation().getX()) > VisionConstants.MAX_ANGLE)
        //   continue;
        // if (Math.abs(inputs[i].estimatedPose.getRotation().getY()) > VisionConstants.MAX_ANGLE)
        //   continue;

        // don't use if there is a only one tag and the ambiguity is too high
        if (inputs[i].tagCount == 1
            && inputs[i].ambiguities.length == 1
            && inputs[i].ambiguities[0] > VisionConstants.SINGLE_TAG_AMBIGUITY_LIMIT) continue;

        double translationalDelta =
            Math.hypot(
                inputs[i].estimatedPose.getX() - robotPose.getX(),
                inputs[i].estimatedPose.getY() - robotPose.getY());

        Matrix<N3, N1> stddevs =
            getEstimationStdDevs(
                inputs[i].estimatedPose,
                inputs[i].tagIds,
                inputs[i].ambiguities,
                inputs[i].distToCams,
                translationalDelta,
                isRobotOverBump);

        addedPose = true;
        Logger.recordOutput(
            "/Vision/" + limelightNames[i] + "/Raw Vision", inputs[i].estimatedPose);
        Logger.recordOutput(
            "/Vision/" + limelightNames[i] + "/Vision Timestamp", inputs[i].timestampSeconds);
        Logger.recordOutput(
            "/Vision/" + limelightNames[i] + "/Vision Std Dev",
            new double[] {stddevs.get(0, 0), stddevs.get(1, 0), stddevs.get(2, 0)});

        estimates.add(
            new PoseEstimate(inputs[i].estimatedPose, inputs[i].timestampSeconds, stddevs));
      }
      if (!addedPose) {
        Logger.recordOutput(
            "/Vision/" + limelightNames[i] + "/Raw Vision",
            new Pose2d(-1000, -1000, new Rotation2d()));
      }
    }
    // logging detected tags
    Logger.recordOutput("Detected Tag Poses", detectedTagPoses.toArray(Pose3d[]::new));

    return estimates.toArray(PoseEstimate[]::new);
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose,
      int[] tagIds,
      double[] ambiguities,
      double[] distToCams,
      double translationalDelta,
      boolean isRobotOverBump) {
    // if the robot is over a bump, fully trust vision
    // if (isRobotOverBump == true) {
    //   return VecBuilder.fill(0.0, 0.0, 0.0);
    // }

    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    int numTags = 0;
    double avgDist = 0;
    for (int i = 0; i < tagIds.length; i++) {
      // if nonexistent tag, ignore
      var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tagIds[i]);
      if (tagPose.isEmpty()) continue;

      numTags++;
      avgDist +=
          distToCams[i]; // tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;

    // decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.MULTIPLE_TAG_STD_DEVS;
    // increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 7)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
      // increase std devs based on ambiguity if only one tag is visible
      if (numTags == 1) {
        estStdDevs = estStdDevs.times(1 + (ambiguities[0] * ambiguities[0]) * 2);
      }
    }

    if (numTags == 1 && translationalDelta > VisionConstants.MAX_SINGLE_TAG_TRANSLATIONAL_DELTA) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    if (VisionConstants.IGNORE_YAW) estStdDevs.set(2, 0, Double.MAX_VALUE);

    return estStdDevs;
  }

  public void resetSimState(Pose2d robotPose) {
    for (int i = 0; i < ios.length; i++) {
      inputs[i].estimateIsPresent = false;
      inputs[i].tagCount = 0;
      inputs[i].tagIds = new int[0];
      ios[i].resetSimState(robotPose);
    }
  }
}
