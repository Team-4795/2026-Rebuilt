package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  DCMotorSim topMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.001, ShooterConstants.GEARING),
          DCMotor.getKrakenX44(1));

  DCMotorSim bottomMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.001, ShooterConstants.GEARING),
          DCMotor.getKrakenX44(1));

  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.09);
  PIDController controller = new PIDController(0.5, 0, 0);

  private double topMotorVel = 0.0;
  private double bottomMotorVel = 0.0;

  private double topVolts = 0.0;
  private double bottomVolts = 0.0;

  @Override
  public void setVelocityRPS(double velocityRPS) {
    this.topMotorVel = velocityRPS;
    this.bottomMotorVel = velocityRPS;
  }

  @Override
  public double getTopRPS() {
    return topMotor.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public double getBottomRPS() {
    return bottomMotor.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public double getGoal() {
    return topMotorVel;
  }

  @Override
  public boolean readyToShoot() {
    return (getGoal() != 0)
        && (getTopRPS() > (getGoal() - 10))
        && (getBottomRPS() > (getGoal() - 10));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topMotor.update(0.02);
    bottomMotor.update(0.02);

    topVolts =
        MathUtil.clamp(
            ff.calculate(topMotorVel)
                + controller.calculate(topMotor.getAngularVelocityRPM() / 60.0, topMotorVel),
            -12,
            12);
    bottomVolts =
        MathUtil.clamp(
            ff.calculate(bottomMotorVel)
                + controller.calculate(bottomMotor.getAngularVelocityRPM() / 60.0, bottomMotorVel),
            -12,
            12);

    topMotor.setInputVoltage(topVolts);
    bottomMotor.setInputVoltage(bottomVolts);

    inputs.goalVelocityRPS = getGoal();

    inputs.bottomShooterVelocityRPS = getBottomRPS();
    inputs.bottomShooterCurrent = bottomMotor.getCurrentDrawAmps();
    inputs.bottomShooterVolts = bottomVolts;

    inputs.topShooterVelocityRPS = getTopRPS();
    inputs.topShooterCurrent = topMotor.getCurrentDrawAmps();
    inputs.topShooterVolts = topVolts;
  }
}
