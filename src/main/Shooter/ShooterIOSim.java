package frc.robot.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    {

    DCMotorSim topMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.001);
    // DCMotorSim bottomMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1,  )
    DCMotorSim bottomMotor = new DCMotorSim(null, null, null);

}
}
