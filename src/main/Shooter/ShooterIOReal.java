package frc.robot.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class ShooterIOReal implements ShooterIO {
    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.leftCanID);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.rightCanID);
    private TalonFXConfiguration configTop = new TalonFXConfiguration();
    private TalonFXConfiguration configBottom = new TalonFXConfiguration();

    public VelocityVoltage controller = new VelocityVoltage();

    public ShooterIOReal() {
        configTop.Slot0.kA = 0;
        configTop.Slot0.kD = 0;
        configTop.Slot0.kG = 0;
        configTop.Slot0.kI = 0;
        configTop.Slot0.kP = 0;
        configTop.Slot0.kS = 0;
        configTop.Slot0.kV = kV;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        


    }
}
