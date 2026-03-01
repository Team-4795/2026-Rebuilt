package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final double margin = 10;
  public static final double minAngle = 0 + margin;
  public static final double maxAngle = 360 - margin;

  // Margin for ready to shoot command
  public static final double marginOfError = 0.075;

  public static final double kP = 3;
  public static final double kI = 0.5;
  public static final double kD = 0;
  public static final double kV = 5.8;
  public static final double kA = 0;
  public static final double kS = 0.3;

  public static final double simkP = 6.5;
  public static final double simkV = 0;

  public static final double maxVelocity = 6;
  public static final double maxAcceleration = 10;
  public static final double expoA = 0;
  public static final double expoV = 0;
  public static final double maxJerk = 0;
  public static final int CAN_ID = 14;

  public static boolean canMove = true;

  public static final double gearing = 60;
  public static final Translation2d OFFSET = new Translation2d(-0.152, -0.165);
  public static final double turretRadiusOffset =
      Math.sqrt(Math.pow(OFFSET.getX(), 2) + Math.pow(OFFSET.getY(), 2));
  public static final double robotRelativeAngleOffset = 0;
  public static final double angleOffset = Units.degreesToRotations(22.98);
}
