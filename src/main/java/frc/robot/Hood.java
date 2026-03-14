package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private double hoodAngle = 0.0;

  public Hood() {}

  public void setAngle(double newAngle) {
    if (newAngle > Constants.HoodConstants.maxAngle) {
      newAngle = Constants.HoodConstants.maxAngle;
    }

    if (newAngle < Constants.HoodConstants.minAngle) {
      newAngle = Constants.HoodConstants.minAngle;
    }

    hoodAngle = newAngle;
  }

  public double getAngle() {
    return hoodAngle;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hood/Angle", hoodAngle);
  }
}
