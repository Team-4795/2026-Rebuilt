package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;
  private PhotonCamera cameraOne;
  private PhotonCamera cameraTwo;
  private PhotonCamera cameraThree;
  private PhotonCameraSim cameraSimOne;
  private PhotonCameraSim cameraSimTwo;
  private PhotonCameraSim cameraSimThree;

  public VisionIOSim() {
    if(Constants.isVisionSim) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);

      cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(78));
      cameraProp.setCalibError(0.38, 0.2);
      cameraProp.setFPS(30);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      // Add all three cameras to sim
      cameraOne = new PhotonCamera(VisionConstants.CAM_NAMES[0]);
      cameraSimOne = new PhotonCameraSim(cameraOne, cameraProp);
      cameraSimOne.setMaxSightRange(8);

      cameraTwo = new PhotonCamera(VisionConstants.CAM_NAMES[1]);
      cameraSimTwo = new PhotonCameraSim(cameraTwo, cameraProp);
      cameraSimTwo.setMaxSightRange(8);

      cameraThree = new PhotonCamera(VisionConstants.CAM_NAMES[2]);
      cameraSimThree = new PhotonCameraSim(cameraThree, cameraProp);
      cameraSimThree.setMaxSightRange(8);

      visionSim.addCamera(cameraSimOne, VisionConstants.CAM_POSES[0]);
      visionSim.addCamera(cameraSimTwo, VisionConstants.CAM_POSES[1]);
      visionSim.addCamera(cameraSimThree, VisionConstants.CAM_POSES[2]);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) { 
    if(Constants.isVisionSim) {
      visionSim.update(Drive.getInstance().getPose());
      visionSim.getDebugField();

      Logger.recordOutput(
          "Vision/Camera Poses/Front Cam Pose", visionSim.getCameraPose(cameraSimOne).get());
      Logger.recordOutput(
          "Vision/Camera Poses/Left Cam Pose", visionSim.getCameraPose(cameraSimTwo).get());
      Logger.recordOutput(
          "Vision/Camera Poses/Right Cam Pose", visionSim.getCameraPose(cameraSimThree).get());
    }
  }
}
