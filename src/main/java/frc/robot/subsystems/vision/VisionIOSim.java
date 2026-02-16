package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator estimator;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final String cameraName;
  public int focusTag = 0;

  public VisionIOSim(
      String cameraName, Transform3d robotToCam, Supplier<Pose2d> robotPoseSupplier) {
    this.cameraName = cameraName;
    this.robotPoseSupplier = robotPoseSupplier;

    visionSim = new VisionSystemSim(cameraName);
    visionSim.addAprilTags(TAG_LAYOUT);

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(cameraDiagonalFOVDegrees));
    cameraProp.setCalibError(0.8, 0.08);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    camera = new PhotonCamera(cameraName);
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    estimator =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    visionSim.addCamera(cameraSim, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = cameraName;

    visionSim.update(robotPoseSupplier.get());

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      PhotonPipelineResult result = results.get(results.size() - 1);

      var optionalEstimate = estimator.update(result);
      if (optionalEstimate.isPresent()) {
        inputs.tagCount = result.getTargets().size();

        List<PhotonTrackedTarget> tags = result.getTargets();
        int[] tagIds = new int[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
          tagIds[i] = tags.get(i).getFiducialId();
        }
        inputs.tagIds = tagIds;

        inputs.estimatedPose = optionalEstimate.get().estimatedPose.toPose2d();
        inputs.timestampSeconds = optionalEstimate.get().timestampSeconds;
        inputs.estimateIsPresent = true;
      } else {
        inputs.estimateIsPresent = false;
        inputs.tagIds = new int[0];
      }
    } else {
      inputs.estimateIsPresent = false;
      inputs.tagIds = new int[0];
    }
  }
}
