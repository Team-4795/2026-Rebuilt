package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterHoodConstants {
  public static final int CAN_ID = 17;

  public static final int CURRENT_LIMIT = 80;

  public static final double kP = 0;
  public static final int kI = 0;
  public static final int kD = 0;
  public static final int kS = 0;
  public static final double kV = 0.0;
  public static final int kA = 0;

  public static final int GEARING = 30; // might not be right

  // Rotations
  public static final double margin = 0.05;
  public static final double maxAngle = 0.5 - margin;
  public static final double minAngle = 0.0 + margin;

  // Margin for ready to shoot command
  public static final double marginOfError = 0.01;

  public static final double maxVelocity = 10;
  public static final double maxAcceleration = 10;
  public static final double maxJerk = 0;

  // change when testing
  public static final double boxXMultiplier = 0.15;
  public static final double boxYMultiplier = 0.05;

  public static final InterpolatingDoubleTreeMap shooterHoodMap = new InterpolatingDoubleTreeMap();

  // distance, rotations
  static {
    shooterHoodMap.put(1.0, 10.0);
    shooterHoodMap.put(2.0, 0.5);
  }
}
