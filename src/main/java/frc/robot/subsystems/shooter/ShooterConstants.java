package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  // Change later
  public static final int TOP_CAN_ID = 17;
  public static final int BOTTOM_CAN_ID = 19;

  public static final int CURRENT_LIMIT = 40;

  public static final double minVel = 0.0;
  public static final double maxVel = 100.0;

  public static final double kP = 2.8;
  public static final double kI = 0;
  public static final double kD = 0.0;
  public static final double kS = 0;
  public static final double kV = 0.07;
  public static final double kA = 0;
  public static final double RPM = 50;

  public static final double GEARING = 2.0 / 3.0;

  // motion magic constants
  public static final double MM_ACCELERATION = 75;
  public static final double MM_JERK = 150;

  public static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

  // Distance, RPM
  static {
    shooterMap.put(0.0, 0.0);
    shooterMap.put(1.0, 20.0);
    shooterMap.put(2.0, 30.0);
  }
}
