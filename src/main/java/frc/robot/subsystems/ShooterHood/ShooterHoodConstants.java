package frc.robot.subsystems.ShooterHood;

public final class ShooterHoodConstants {
  // Change later
  public static final int CAN_ID = 16;

  public static final int CURRENT_LIMIT = 40;

  public static final int kP = 0;
  public static final int kI = 0;
  public static final int kD = 0;
  public static final int kS = 0;
  public static final int kV = 0;
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
}
