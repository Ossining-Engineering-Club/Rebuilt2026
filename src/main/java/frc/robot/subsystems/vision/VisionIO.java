package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public int tagCount = 0;
    public int[] tagIds = new int[0];
    public Pose2d estimatedPose = new Pose2d();
    public double timestampSeconds = 0;
    public boolean estimateIsPresent = false;
    public String cameraName = "";
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
