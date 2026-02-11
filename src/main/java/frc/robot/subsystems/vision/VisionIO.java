package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    Pose3d[] pose = new Pose3d[] {};
    double[] timestamp = new double[] {};
    int[] tags = new int[] {};

    double poseAmbiguity = 0.0;

    int pipelineIndex = 10;
    String sortMode = "";
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
