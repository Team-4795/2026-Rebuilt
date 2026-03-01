package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterHoodConstants {
  public static final int CAN_ID = 17;

  public static final int CURRENT_LIMIT = 80;

  public static final double kP = 40;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.5;
  public static final double kV = 13.2;
  public static final double kA = 0;

  public static final int GEARING = 140; // might not be right

  // Rotations
  public static final double maxAngle = 0.0;
  public static final double minAngle = -0.1;

  // Margin for ready to shoot command
  public static final double marginOfError = 0.01;

  public static final double maxVelocity = 10;
  public static final double maxAcceleration = 10;
  public static final double maxJerk = 0;

  // change when testing
  public static final double boxXMultiplier = 0.15;
  public static final double boxYMultiplier = 0.05;

  public static final InterpolatingDoubleTreeMap shooterHoodHubMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterHoodShuttlingMap =
      new InterpolatingDoubleTreeMap();

  // distance, rotations
  static {
    shooterHoodHubMap.put(4.83, -0.068);
    shooterHoodHubMap.put(4.33, -0.067);
    shooterHoodHubMap.put(3.8, -0.06);
    shooterHoodHubMap.put(3.24, -0.049);
    shooterHoodHubMap.put(2.75, -0.0375);
    shooterHoodHubMap.put(2.23, -0.025);
    shooterHoodHubMap.put(1.754, -0.014);
    shooterHoodHubMap.put(1.16, -0.01);

    // Shuttling
    shooterHoodShuttlingMap.put(5.246212, -0.07);
    shooterHoodShuttlingMap.put(6.4790482, -0.08);
    shooterHoodShuttlingMap.put(7.329184, -0.085);
  }
}
