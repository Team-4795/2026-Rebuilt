package frc.robot.subsystems.Shooter;

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

  public static final double GEARING = 2.0 / 3.0;

  // motion magic constants
  public static final double MM_ACCELERATION = 75;
  public static final double MM_JERK = 150;
}
