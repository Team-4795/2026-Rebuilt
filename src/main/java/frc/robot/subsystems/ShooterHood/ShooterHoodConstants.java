package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterHoodConstants {
  public static final int CAN_ID = 17;

  public static final int CURRENT_LIMIT = 80;

  public static final double kP = 1280;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;

  public static final int GEARING = 140; // might not be right

  // Rotations
  public static final double maxAngle = 0.0;
  public static final double minAngle = -0.103;

  // Margin for ready to shoot command
  public static final double marginOfError = 0.01;

  public static final double maxVelocity = 10;
  public static final double maxAcceleration = 10;
  public static final double maxJerk = 0;

  // change when testing
  public static final double boxXMultiplier = 0.3;

  public static double outreachAngle = 0;

  public static final InterpolatingDoubleTreeMap shooterHoodHubMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterHoodShuttlingMap =
      new InterpolatingDoubleTreeMap();

  // distance, rotations
  static {
    // Shooting
    shooterHoodHubMap.put(1.628, -0.015);
    shooterHoodHubMap.put(2.233, -0.0325);
    shooterHoodHubMap.put(2.696, -0.0325);
    shooterHoodHubMap.put(3.200, -0.0425);
    shooterHoodHubMap.put(4.06, -0.046);
    shooterHoodHubMap.put(4.61, -0.047);
    shooterHoodHubMap.put(5.124, -0.053);
    shooterHoodHubMap.put(5.77, -0.058);
    shooterHoodHubMap.put(6.2, -0.06);

    // Shuttling
    shooterHoodShuttlingMap.put(3.082, -0.08);
    shooterHoodShuttlingMap.put(4.232, -0.074);
    shooterHoodShuttlingMap.put(5.222, -0.07);
    shooterHoodShuttlingMap.put(5.97, -0.075);
    shooterHoodShuttlingMap.put(7.14, -0.075);
    shooterHoodShuttlingMap.put(7.329184, -0.085);
    shooterHoodShuttlingMap.put(10.0, -0.09);
    shooterHoodShuttlingMap.put(12.0, -0.103);
    shooterHoodShuttlingMap.put(14.0, -0.103);
  }
}
