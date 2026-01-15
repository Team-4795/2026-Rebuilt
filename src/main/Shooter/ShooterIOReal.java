package frc.robot.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class ShooterIOReal implements ShooterIO {
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.leftCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFXConfigurator config = new TalonFXConfigurator();

    public VelocityVoltage controller = new VelocityVoltage();

    public ShooterIOReal() {

    }
}
