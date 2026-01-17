package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  {
    DCMotorSim topMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.001, ShooterConstants.GEARING),
            DCMotor.getKrakenX60(1),
            0.001);
    // DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1,  )
    DCMotorSim bottomMotor = new DCMotorSim(null, null, null);

    SimpleMotorFeedforward ffd = new SimpleMotorFeedforward(0, ShooterConstants.kV / 0);
    PIDController controller = new PIDController(ShooterConstants.kP / 0, 0, 0);

    final double topSpeed = 0.0;
    final double bottomSpeed = 0.0;
  }
}
