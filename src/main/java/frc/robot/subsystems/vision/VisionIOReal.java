package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOReal implements VisionIO {
  PhotonCamera camera;
  PhotonPoseEstimator estimator;

  PhotonTargetSortMode sortMode;
  List<PhotonPipelineResult> result;

  public VisionIOReal(int camIndex) {
    camera = new PhotonCamera(VisionConstants.CAM_NAMES[camIndex]);

    estimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
            VisionConstants.CAM_POSES[camIndex]);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    sortMode = PhotonTargetSortMode.Largest;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.pipelineIndex = camera.getPipelineIndex();
    inputs.sortMode = sortMode.toString();

    result = camera.getAllUnreadResults();

    for (int i = 0; i < result.size(); i++) {
      if (result.get(i).getBestTarget() != null) {
        inputs.poseAmbiguity = result.get(i).getBestTarget().getPoseAmbiguity();
      }

      estimator
          .update(result.get(i), camera.getCameraMatrix(), camera.getDistCoeffs())
          .ifPresentOrElse(
              (pose) -> {
                inputs.pose = new Pose3d[] {pose.estimatedPose};
                inputs.timestamp = new double[] {pose.timestampSeconds};
                inputs.tags =
                    pose.targetsUsed.stream()
                        .mapToInt(
                            (target) -> {
                              return target.getFiducialId();
                            })
                        .toArray();
              },
              () -> {
                inputs.pose = new Pose3d[] {};
                inputs.timestamp = new double[] {};
                inputs.tags = new int[] {};
              });
    }
  }
}
