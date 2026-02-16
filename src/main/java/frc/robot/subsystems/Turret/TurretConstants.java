package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {
  public static final double margin = 3;
  public static final double minAngle = 0 + margin;
  public static final double maxAngle = 360 - margin;

  // Margin for ready to shoot command
  public static final double marginOfError = 0.01;

  public static final double kP = 5;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kV = 5.3;
  public static final double kA = 0.12;
  public static final double kS = 0;

  public static final double simkP = 6.5;
  public static final double simkV = 0;

  public static final double maxVelocity = 6;
  public static final double maxAcceleration = 10;
  public static final double expoA = 0; 
  public static final double expoV = 0;
  public static final double maxJerk = 0;
  public static final int CAN_ID = 20;

  public static final double gearing = 60;
  public static final Translation2d OFFSET = new Translation2d(0.152, -0.165);
  public static final double angleOffset = 0;
}
