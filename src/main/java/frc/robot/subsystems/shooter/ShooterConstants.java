package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  public static final int TOP_CAN_ID = 15;
  public static final int BOTTOM_CAN_ID = 16;

  public static final int CURRENT_LIMIT = 120;

  public static final double minVel = 0.0;
  public static final double maxVel = 75.0;

  // Margin for ready to shoot command
  public static final double marginOfError = 1.0;

  public static final double kP = 100000;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final double RPM = 60;

  public static final double GEARING = 1.0 / 2.0;

  // motion magic constants
  public static final double MM_ACCELERATION = 999;
  public static final double MM_JERK = 999;

  public static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

  // Distance, RPM
  static {
    shooterMap.put(3.742609, 60.0);
    shooterMap.put(4.857, 61.0);
  }
}
