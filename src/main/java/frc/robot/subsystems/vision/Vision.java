package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private VisionIO[] io;
  private VisionIOInputsAutoLogged inputs[];

  // change to match number of cameras
  private boolean[] shouldUpdate = new boolean[] {true, true, true};

  public static Vision instance;

  public static Vision getInstance() {
    return instance;
  }

  public static Vision initialize(VisionIO... io) {
    if (instance == null) {
      instance = new Vision(io);
    }
    return instance;
  }

  public Vision(VisionIO visionIO[]) {
    io = visionIO;
    inputs = new VisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  public void toggleShouldUpdate() {
    for (int i = 0; i < io.length; i++) {
      shouldUpdate[i] = !shouldUpdate[i];
    }
  }

  public boolean isVisionUpdating() {
    return shouldUpdate[0];
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + VisionConstants.CAM_NAMES[i], inputs[i]);
      Logger.recordOutput(
          "Vision/" + VisionConstants.CAM_NAMES[i] + "/is updating", isVisionUpdating());
    }

    for (int i = 0; i < io.length; i++) { // Iterate through all cams
      for (int p = 0; p < inputs[i].pose.length; p++) { // Iterate through estimated poses
        Pose3d robotPose = inputs[i].pose[p];

        // throw out bad data that's outside margins
        if (robotPose.getX() < -VisionConstants.BORDER_MARGIN
            || robotPose.getX()
                > Constants.FieldConstants.fieldLength + VisionConstants.BORDER_MARGIN
            || robotPose.getY() < -VisionConstants.BORDER_MARGIN
            || robotPose.getY()
                > Constants.FieldConstants.fieldWidth + VisionConstants.BORDER_MARGIN
            || robotPose.getZ() < -VisionConstants.Z_MARGIN
            || robotPose.getZ() > VisionConstants.Z_MARGIN) continue;

        List<Pose3d> tagPoses = new ArrayList<>();

        // ADD TAGS TO CONSIDER
        for (int tag : inputs[i].tags) {
          // Exclude tower and feeder tags
          if (tag >= 13 && tag <= 16) continue;
          if (tag >= 29 && tag <= 32) continue;

          VisionConstants.FIELD_LAYOUT.getTagPose(tag).ifPresent(tagPoses::add);

          // Hub only if you see non hub tags (practice field only)
          if (tag == 1 || tag == 12 || tag == 6 || tag == 7 || tag == 15 || tag == 16 || tag == 13
              || tag == 14 || tag == 22 || tag == 23 || tag == 17 || tag == 28 || tag == 29
              || tag == 30 || tag == 31 || tag == 32) VisionConstants.hubOnlyMode = true;
          else VisionConstants.hubOnlyMode = false;
        }
        if (tagPoses.isEmpty()) continue;

        // calculate average distance
        double distance = 0.0;
        for (var tag : tagPoses) {
          distance += tag.getTranslation().getDistance(robotPose.getTranslation());
        }
        distance /= tagPoses.size();

        // calculate stdevs based on distance/# of tags
        double xyStdDev =
            (tagPoses.size() == 1
                    ? VisionConstants.XY_SINGLE_STDEV
                    : VisionConstants.XY_MULTIPLE_STDEV)
                * Math.pow(distance, 2);
        var stddevs = VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(100));

        Logger.recordOutput("Vision/" + VisionConstants.CAM_NAMES[i] + "/Avg distance", distance);
        Logger.recordOutput("Vision/" + VisionConstants.CAM_NAMES[i] + "/xy std dev", xyStdDev);
        Logger.recordOutput("Vision/" + VisionConstants.CAM_NAMES[i] + "/Robot Pose", robotPose);

        if (shouldUpdate[i]) {
          Drive.getInstance()
              .addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
        }
      }
    }
  }
}
