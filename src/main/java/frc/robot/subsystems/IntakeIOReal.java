package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class IntakeIOReal implements IntakeIO {
  // two motors on each side of the spinny bar thing
  private final SparkFlex intakeMotorA =
      new SparkFlex(IntakeConstants.canIDIntakeA, MotorType.kBrushless);
  private final SparkFlex intakeMotorB =
      new SparkFlex(IntakeConstants.canIDIntakeB, MotorType.kBrushless);

  // two motors to extend intake
  private final SparkFlex intakeDeployMotorA =
      new SparkFlex(IntakeConstants.canIDDeployA, MotorType.kBrushless);
  private final SparkFlex intakeDeployMotorB =
      new SparkFlex(IntakeConstants.canIDDeployB, MotorType.kBrushless);

  private SparkFlexConfig intakeConfig = new SparkFlexConfig();
  private SparkFlexConfig deployConfig = new SparkFlexConfig();
  private RelativeEncoder encoder = intakeDeployMotorA.getEncoder();

  private ArmFeedforward ffmodel =
      new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);
  private PIDController controller =
      new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          IntakeConstants.MAX_VELOCITY, IntakeConstants.MAX_ACCELERATION);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State setpoint;

  private double intakeVolts = 0;
  private double deployVolts = 0;

  public IntakeIOReal() {
    // Deploy motor config
    intakeDeployMotorA.clearFaults();
    intakeDeployMotorB.clearFaults();

    deployConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    deployConfig.idleMode(IdleMode.kBrake);

    deployConfig.encoder.positionConversionFactor(IntakeConstants.GEARING_DEPLOY);
    deployConfig.encoder.velocityConversionFactor(IntakeConstants.GEARING_DEPLOY / 60);
    deployConfig.encoder.quadratureMeasurementPeriod(20);

    deployConfig.softLimit.forwardSoftLimitEnabled(true);
    deployConfig.softLimit.reverseSoftLimitEnabled(false);
    deployConfig.softLimit.forwardSoftLimit(IntakeConstants.deployMaxAngle);
    deployConfig.softLimit.reverseSoftLimit(IntakeConstants.deployMinAngle);

    deployConfig.voltageCompensation(12.0);
    deployConfig.inverted(false);

    intakeDeployMotorA.configure(
        deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeDeployMotorB.configure(
        deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Intake motor config
    intakeMotorA.clearFaults();
    intakeMotorB.clearFaults();

    intakeConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeConfig.idleMode(IdleMode.kCoast);

    intakeMotorA.setCANTimeout(20);
    intakeMotorB.setCANTimeout(20);

    intakeMotorA.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorB.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setGoal(double angleRot) {
    double angle = Units.rotationsToRadians(angleRot);
    if (angle != goal.position) {
      setpoint = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
      goal = new TrapezoidProfile.State(angle, 0);
    }
  }

  @Override
  public void updateMotionProfile() {
    setpoint = profile.calculate(0.02, setpoint, goal);
    double ffvolts = ffmodel.calculate(encoder.getPosition(), setpoint.velocity);
    double pidvolts = controller.calculate(encoder.getPosition(), setpoint.position);

    setDeployVoltage(ffvolts + pidvolts);
  }

  @Override
  public void setDeployVoltage(double v) {
    intakeVolts = v;
    intakeDeployMotorA.setVoltage(intakeVolts);
    intakeDeployMotorB.setVoltage(intakeVolts);
  }

  @Override
  public void setIntakeVoltage(double v) {
    deployVolts = v;
    intakeMotorA.setVoltage(deployVolts);
    intakeMotorB.setVoltage(deployVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltageA = intakeVolts;
    inputs.angularVelocityRPSA = intakeMotorA.getEncoder().getVelocity();
    inputs.currentAmpsA = intakeMotorA.getOutputCurrent();

    inputs.voltageB = intakeVolts;
    inputs.angularVelocityRPSB = intakeMotorB.getEncoder().getVelocity();
    inputs.currentAmpsB = intakeMotorB.getOutputCurrent();

    inputs.deployMotorVoltage = deployVolts;
    inputs.deployMotorPosition = encoder.getPosition();
    inputs.deployMotorVelocity = encoder.getVelocity();
    inputs.deployMotorGoal = goal.position;
    inputs.deployMotorSetpoint = setpoint.position;
  }
}
