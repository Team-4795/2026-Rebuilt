// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.vision.VisionConstants;

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
    public static double fieldlWidth = VisionConstants.FIELD_LAYOUT.getFieldWidth();

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
  }
}
