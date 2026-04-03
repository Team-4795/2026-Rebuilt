package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  public static final int TOP_CAN_ID = 15;
  public static final int BOTTOM_CAN_ID = 16;

  public static final int STATOR_CURRENT_LIMIT = 120;
  public static final int SUPPLY_CURRENT_LIMIT = 120;
  public static final int SUPPLY_CURRENT_LIMIT_LOWER = 60;

  public static final double minVel = -30.0;
  public static final double maxVel = 100.0;

  // Margin for ready to shoot command
  public static final double marginOfError = 4.0;

  public static final double kP = 12;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final double RPS = 60;

  public static final double GEARING = 18.0 / 30.0;

  // motion magic constants
  public static final double MM_ACCELERATION = 999;
  public static final double MM_JERK = 999;

  public static final InterpolatingDoubleTreeMap shooterVelocityHubMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterVelocityShuttlingMap =
      new InterpolatingDoubleTreeMap();

  // Distance, RPM
  static {
    // shooterVelocityHubMap.put(1.16, 55.0);
    // shooterVelocityHubMap.put(1.754, 57.0);
    // shooterVelocityHubMap.put(2.23, 60.0);
    // shooterVelocityHubMap.put(2.75, 64.0);
    // shooterVelocityHubMap.put(3.24, 64.0);
    // shooterVelocityHubMap.put(4.04, 69.0);
    // shooterVelocityHubMap.put(4.325, 71.75);
    // shooterVelocityHubMap.put(4.85, 72.75);
    // shooterVelocityHubMap.put(5.47, 74.5);
    shooterVelocityHubMap.put(1.628, 55.0);
    shooterVelocityHubMap.put(2.233, 55.0);
    shooterVelocityHubMap.put(2.696, 59.6);
    shooterVelocityHubMap.put(3.200, 61.5);
    shooterVelocityHubMap.put(4.06, 67.25);
    shooterVelocityHubMap.put(4.61, 68.25);
    shooterVelocityHubMap.put(5.124, 73.25);
    shooterVelocityHubMap.put(5.77, 74.75);
    shooterVelocityHubMap.put(6.2, 76.0);

    // Shuttling
    // shooterVelocityShuttlingMap.put(5.246212, 70.0);
    // shooterVelocityShuttlingMap.put(6.479048, 73.0);
    // shooterVelocityShuttlingMap.put(10.0, 95.0);

    shooterVelocityShuttlingMap.put(3.082, 45.0);
    shooterVelocityShuttlingMap.put(4.232, 54.0);
    shooterVelocityShuttlingMap.put(5.222, 64.0);
    shooterVelocityShuttlingMap.put(5.97, 70.0);
    shooterVelocityShuttlingMap.put(7.14, 78.0);
    shooterVelocityShuttlingMap.put(7.329184, 78.0);
  }
}
