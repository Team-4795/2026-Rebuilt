package frc.robot.subsystems.IntakeDeploy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeDeployIOSim implements IntakeDeployIO {
  private ArmFeedforward ffmodel = new ArmFeedforward(0.05, IntakeDeployConstants.kG, 0.9);
  private PIDController controller =
      new PIDController(2, IntakeDeployConstants.kI, IntakeDeployConstants.kD);

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          IntakeDeployConstants.MAX_VELOCITY, IntakeDeployConstants.MAX_ACCELERATION);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  DCMotorSim motorDeployA =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.001, IntakeDeployConstants.GEARING_DEPLOY),
          DCMotor.getKrakenX60(1));

  DCMotorSim motorDeployB =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.001, IntakeDeployConstants.GEARING_DEPLOY),
          DCMotor.getKrakenX60(1));

  @Override
  public void setVoltage(double v) {
    motorDeployA.setInputVoltage(v);
    motorDeployB.setInputVoltage(v);
  }

  @Override
  public void setGoal(double angle) {
    if (angle != goal.position) {
      setpoint =
          new TrapezoidProfile.State(
              motorDeployA.getAngularPositionRotations(),
              motorDeployA.getAngularVelocityRPM() / 60.0); // change if something goes funny
      goal = new TrapezoidProfile.State(angle, 0);
    }
  }

  @Override
  public void updateMotionProfile() {
    setpoint = profile.calculate(0.02, setpoint, goal);
    double ffvolts =
        ffmodel.calculate(
            Units.rotationsToRadians(
                motorDeployA.getAngularPositionRotations() - IntakeDeployConstants.deployOffset),
            Units.rotationsPerMinuteToRadiansPerSecond(setpoint.velocity * 60.0));
    double pidvolts =
        controller.calculate(motorDeployA.getAngularPositionRotations(), setpoint.position);

    setVoltage(ffvolts + pidvolts);
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    motorDeployA.update(0.02);
    motorDeployB.update(0.02);

    inputs.deployMotorPositionA = motorDeployA.getAngularPositionRotations();
    inputs.deployMotorVelocityA = motorDeployA.getAngularVelocityRPM() / 60.0;
    inputs.deployMotorPositionB = motorDeployB.getAngularPositionRotations();
    inputs.deployMotorVelocityB = motorDeployB.getAngularVelocityRPM() / 60.0;

    inputs.deployMotorVoltage = motorDeployA.getInputVoltage();
    inputs.deployMotorGoal = goal.position;
    inputs.deployMotorSetpoint = setpoint.position;
  }
}
