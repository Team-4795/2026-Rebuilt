// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final boolean tuningMode = true;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean isVisionSim = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OIConstants {
    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
    public static final double KAxisDeadband = 0.1;
    public static final double OperatorLAxisDeadband = 0.3;
  }

  public static class FieldConstants {
    public static double fieldLength = VisionConstants.FIELD_LAYOUT.getFieldLength();
    public static double fieldWidth = VisionConstants.FIELD_LAYOUT.getFieldWidth();

    public static double trenchWidth = Units.inchesToMeters(50);

    public static Translation2d redHub = new Translation2d(11.910, 4.060);
    public static Translation2d redLeftTrench =
        VisionConstants.FIELD_LAYOUT.getTagPose(7).get().getTranslation().toTranslation2d();
    public static Translation2d redRightTrench =
        VisionConstants.FIELD_LAYOUT.getTagPose(12).get().getTranslation().toTranslation2d();

    public static Translation2d blueHub = new Translation2d(fieldLength - 11.910, 4.060);
    public static Translation2d blueLeftTrench =
        VisionConstants.FIELD_LAYOUT.getTagPose(23).get().getTranslation().toTranslation2d();
    public static Translation2d blueRightTrench =
        VisionConstants.FIELD_LAYOUT.getTagPose(28).get().getTranslation().toTranslation2d();

    public static ArrayList<Translation2d> trenchList = new ArrayList<Translation2d>();

    public static Translation2d[] blueShuttleZoneOne = new Translation2d[4];
    public static Translation2d[] blueShuttleZoneTwo = new Translation2d[4];
    public static Translation2d[] blueShuttleZoneThree = new Translation2d[4];
    public static Translation2d[] blueShuttleZoneFour = new Translation2d[4];
    public static Translation2d[] blueShuttleZoneFive = new Translation2d[4];

    public static Translation2d[] redShuttleZoneOne = new Translation2d[4];
    public static Translation2d[] redShuttleZoneTwo = new Translation2d[4];
    public static Translation2d[] redShuttleZoneThree = new Translation2d[4];
    public static Translation2d[] redShuttleZoneFour = new Translation2d[4];
    public static Translation2d[] redShuttleZoneFive = new Translation2d[4];

    public static void initConstants() {
      double shuttlingDeadzone = 0.5;

      trenchList.add(redLeftTrench);
      trenchList.add(redRightTrench);
      trenchList.add(blueLeftTrench);
      trenchList.add(blueRightTrench);

      // Blue Zone One
      blueShuttleZoneOne[0] = new Translation2d(blueLeftTrench.getX(), fieldWidth / 2.0);
      blueShuttleZoneOne[1] = new Translation2d(redRightTrench.getX(), fieldWidth / 2.0);
      blueShuttleZoneOne[2] = new Translation2d(blueLeftTrench.getX(), 0);
      blueShuttleZoneOne[3] = new Translation2d(redRightTrench.getX(), 0);

      // Blue Zone Two
      for (int i = 0; i < 4; i++) {
        blueShuttleZoneTwo[i] = blueShuttleZoneOne[i].plus(new Translation2d(0, fieldWidth / 2.0));
      }

      // Blue Zone Three
      blueShuttleZoneThree[0] = new Translation2d(redLeftTrench.getX(), 0);
      blueShuttleZoneThree[1] = new Translation2d(fieldLength, 0);
      blueShuttleZoneThree[2] =
          new Translation2d(fieldLength, fieldWidth / 2.0 - shuttlingDeadzone);
      blueShuttleZoneThree[3] =
          new Translation2d(redLeftTrench.getX(), fieldWidth / 2.0 - shuttlingDeadzone);

      // Blue Zone Four
      for (int i = 0; i < 4; i++) {
        blueShuttleZoneFour[i] =
            blueShuttleZoneThree[i].plus(
                new Translation2d(0, fieldWidth / 2.0 + shuttlingDeadzone));
      }

      // Red Zones
      for (int i = 0; i < 4; i++) {
        redShuttleZoneOne[i] =
            blueShuttleZoneOne[i].rotateAround(
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0), new Rotation2d(Math.PI));
        redShuttleZoneTwo[i] =
            blueShuttleZoneTwo[i].rotateAround(
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0), new Rotation2d(Math.PI));
        redShuttleZoneThree[i] =
            blueShuttleZoneThree[i].rotateAround(
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0), new Rotation2d(Math.PI));
        redShuttleZoneFour[i] =
            blueShuttleZoneFour[i].rotateAround(
                new Translation2d(fieldLength / 2.0, fieldWidth / 2.0), new Rotation2d(Math.PI));
      }
    }
  }
}
