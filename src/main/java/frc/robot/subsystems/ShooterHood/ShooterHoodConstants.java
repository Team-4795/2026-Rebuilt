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

  public static final InterpolatingDoubleTreeMap shooterHoodMap = new InterpolatingDoubleTreeMap();

  // distance, rotations
  static {
    // shooterHoodMap.put(4.250619, -0.06); need rps of 63 as well
    // shooterHoodMap.put(3.680411, -0.04);
    // shooterHoodMap.put(3.08628, -0.03);
    // shooterHoodMap.put(2.408265, -0.017);
    shooterHoodMap.put(4.83, -0.068);
    shooterHoodMap.put(4.33, -0.067);
    shooterHoodMap.put(3.8, -0.06);
    shooterHoodMap.put(3.24, -0.049);
    shooterHoodMap.put(2.75, -0.0375);
    shooterHoodMap.put(2.23, -0.025);
    shooterHoodMap.put(1.754, -0.014);
    shooterHoodMap.put(1.16, -0.01);
  }
}
