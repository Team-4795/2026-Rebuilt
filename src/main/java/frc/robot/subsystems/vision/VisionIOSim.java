package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;

  public VisionIOSim() {
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);

    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(78));
    cameraProp.setCalibError(0.38, 0.2);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    // Add all three cameras to sim
    PhotonCamera cameraOne = new PhotonCamera(VisionConstants.CAM_NAMES[0]);
    PhotonCameraSim cameraSimOne = new PhotonCameraSim(cameraOne, cameraProp);

    PhotonCamera cameraTwo = new PhotonCamera(VisionConstants.CAM_NAMES[1]);
    PhotonCameraSim cameraSimTwo = new PhotonCameraSim(cameraTwo, cameraProp);

    PhotonCamera cameraThree = new PhotonCamera(VisionConstants.CAM_NAMES[2]);
    PhotonCameraSim cameraSimThree = new PhotonCameraSim(cameraThree, cameraProp);

    visionSim.addCamera(cameraSimOne, VisionConstants.CAM_POSES[0]);
    visionSim.addCamera(cameraSimTwo, VisionConstants.CAM_POSES[1]);
    visionSim.addCamera(cameraSimThree, VisionConstants.CAM_POSES[2]);

    cameraSimOne.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(Drive.getInstance().getPose());
    visionSim.getDebugField();
  }
}
