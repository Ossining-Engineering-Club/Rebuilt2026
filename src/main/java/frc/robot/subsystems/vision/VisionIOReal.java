package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;

public class VisionIOReal implements VisionIO {
  private final String cameraName;

  public VisionIOReal(String cameraName) {
    this.cameraName = cameraName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = cameraName;

    var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    if (poseEstimate.tagCount == 0) {
      inputs.estimateIsPresent = false;
      inputs.tagIds = new int[0];
    } else {
      inputs.tagCount = poseEstimate.tagCount;

      int[] tagIds = new int[poseEstimate.tagCount];
      for (int i = 0; i < poseEstimate.tagCount; i++) {
        tagIds[i] = poseEstimate.rawFiducials[i].id;
      }
      inputs.tagIds = tagIds;

      inputs.estimatedPose = poseEstimate.pose;
      inputs.timestampSeconds = poseEstimate.timestampSeconds;
      inputs.estimateIsPresent = true;
    }
  }
}
