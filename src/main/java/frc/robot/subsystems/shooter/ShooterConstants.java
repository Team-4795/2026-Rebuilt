package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  public static final int TOP_CAN_ID = 15;
  public static final int BOTTOM_CAN_ID = 16;

  public static final int STATOR_CURRENT_LIMIT = 120;
  public static final int SUPPLY_CURRENT_LIMIT = 80;
  // public static final int SUPPLY_CURRENT_LIMIT_LOWER = 60;

  public static final double minVel = -30.0;
  public static final double maxVel = 130.0;

  // Margin for ready to shoot command
  public static final double marginOfError = 2.0;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kS = 0;
  public static final double kV = 0.2;
  public static final double kA = 0;

  public static final double RPS = 60;

  public static final double GEARING = 18.0 / 30.0;
  // public static final double GEARING = 1.0;

  // motion magic constants
  public static final double MM_ACCELERATION = 999;
  public static final double MM_JERK = 999;

  public static final InterpolatingDoubleTreeMap shooterVelocityHubMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterVelocityShuttlingMap =
      new InterpolatingDoubleTreeMap();

  // Distance, RPM
  static {
    // Shooting
    shooterVelocityHubMap.put(1.628, 54.0);
    shooterVelocityHubMap.put(2.233, 54.0);
    shooterVelocityHubMap.put(2.696, 57.6);
    shooterVelocityHubMap.put(3.200, 60.5);
    shooterVelocityHubMap.put(3.67, 62.15);
    shooterVelocityHubMap.put(4.06, 64.75);
    shooterVelocityHubMap.put(4.61, 68.25);
    shooterVelocityHubMap.put(5.124, 73.25);
    shooterVelocityHubMap.put(5.77, 75.25);
    shooterVelocityHubMap.put(6.2, 77.0);

    // Shuttling
    shooterVelocityShuttlingMap.put(3.082, 45.0);
    shooterVelocityShuttlingMap.put(4.232, 54.0);
    shooterVelocityShuttlingMap.put(5.222, 64.0);
    shooterVelocityShuttlingMap.put(5.97, 70.0);
    shooterVelocityShuttlingMap.put(7.14, 78.0);
    shooterVelocityShuttlingMap.put(7.329184, 78.0);
    shooterVelocityShuttlingMap.put(10.64, 100.0);
    shooterVelocityShuttlingMap.put(13.76, 110.0);
    shooterVelocityShuttlingMap.put(17.76, 123.0);
  }
}
