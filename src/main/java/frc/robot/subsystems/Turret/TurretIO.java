package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double position = 0; 
    public double current = 0; 
    public double volts = 0; 
    public double velocity = 0; 
  }

  public default void setVoltage(double voltage) {}

  public default void updateMotionProfile() {

  }

  public default double getPosition() {
    return 0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setGoal(double goal) {

  } 
}

