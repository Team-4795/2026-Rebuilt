// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    public static double kAxisDeadband = 0.1;
  }

  public static class InterpolatingTree {
    public static final InterpolatingDoubleTreeMap tAirMap = new InterpolatingDoubleTreeMap();

    // Distance, tAir
    static {
      tAirMap.put(1.628, 1.16);
      tAirMap.put(2.233, 1.16);
      tAirMap.put(2.696, 1.28);
      tAirMap.put(3.200, 1.35);
      tAirMap.put(4.06, 1.43);
      tAirMap.put(4.61, 1.55);
      tAirMap.put(5.124, 1.57);
      tAirMap.put(5.77, 1.51);
    }

    // Old values
    // tAirMap.put(2.20, 1.3);
    // tAirMap.put(2.59, 1.26);
    // tAirMap.put(3.03, 1.3);
    // tAirMap.put(3.39, 1.4);
    // tAirMap.put(3.88, 1.5);
    // tAirMap.put(4.39, 1.6);
    // tAirMap.put(5.0, 1.66);
  }

  public static class FieldConstants {
    public static double fieldLength = VisionConstants.FIELD_LAYOUT.getFieldLength();
    public static double fieldWidth = VisionConstants.FIELD_LAYOUT.getFieldWidth();

    public static double trenchWidth = Units.inchesToMeters(50);

    public static Translation2d centerField =
        new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);

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

    public static Translation2d[] shuttleZoneOne = new Translation2d[4];
    public static Translation2d[] shuttleZoneTwo = new Translation2d[4];
    public static Translation2d[] shuttleZoneThree = new Translation2d[4];
    public static Translation2d[] shuttleZoneFour = new Translation2d[4];
    public static Translation2d[] shuttleZoneFive = new Translation2d[4];

    public static Translation2d shuttleTargetOne =
        new Translation2d(blueLeftTrench.getX() - 2, fieldWidth * 0.75);
    public static Translation2d shuttleTargetTwo =
        new Translation2d(blueLeftTrench.getX() - 2, fieldWidth * 0.25);

    public static void initConstants() {
      double shuttlingDeadzone = 0.5;

      trenchList.add(redLeftTrench);
      trenchList.add(redRightTrench);
      trenchList.add(blueLeftTrench);
      trenchList.add(blueRightTrench);

      // Zone One
      shuttleZoneOne[0] = new Translation2d(blueLeftTrench.getX(), fieldWidth / 2.0);
      shuttleZoneOne[1] = new Translation2d(redRightTrench.getX(), fieldWidth / 2.0);
      shuttleZoneOne[2] = new Translation2d(blueLeftTrench.getX(), 0);
      shuttleZoneOne[3] = new Translation2d(redRightTrench.getX(), 0);

      // Zone Two
      for (int i = 0; i < 4; i++) {
        shuttleZoneTwo[i] = shuttleZoneOne[i].plus(new Translation2d(0, fieldWidth / 2.0));
      }

      // Zone Three
      shuttleZoneThree[0] = new Translation2d(redLeftTrench.getX(), 0);
      shuttleZoneThree[1] = new Translation2d(fieldLength, 0);
      shuttleZoneThree[2] = new Translation2d(fieldLength, fieldWidth / 2.0 - shuttlingDeadzone);
      shuttleZoneThree[3] =
          new Translation2d(redLeftTrench.getX(), fieldWidth / 2.0 - shuttlingDeadzone);

      // Zone Four
      for (int i = 0; i < 4; i++) {
        shuttleZoneFour[i] =
            shuttleZoneThree[i].plus(new Translation2d(0, fieldWidth / 2.0 + shuttlingDeadzone));
      }

      // Zone Five
      shuttleZoneFive[0] =
          new Translation2d(redLeftTrench.getX(), centerField.getY() - shuttlingDeadzone);
      shuttleZoneFive[1] = new Translation2d(fieldLength, centerField.getY() - shuttlingDeadzone);
      shuttleZoneFive[2] = new Translation2d(fieldLength, centerField.getY() + shuttlingDeadzone);
      shuttleZoneFive[3] =
          new Translation2d(redLeftTrench.getX(), centerField.getY() + shuttlingDeadzone);
    }
  }
}
