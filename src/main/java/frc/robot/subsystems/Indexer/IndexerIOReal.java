package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class IndexerIOReal implements IndexerIO {
     private final SparkFlex indexerMotor = new SparkFlex(IndexerConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder encoder = indexerMotor.getEncoder();
    
    private SparkFlexConfig config = new SparkFlexConfig();

    public IndexerIOReal() {
        indexerMotor.clearFaults();
        config.smartCurrentLimit(IndexerConstants.currentLimit);
        config.idleMode(IdleMode.kCoast);
        indexerMotor.setCANTimeout(20);
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IndexerIOInputs inputs) {
        inputs.angularVelocityRPM = encoder.getVelocity();
        inputs.angularPositionRot = encoder.getPosition();
        inputs.currentAmps = indexerMotor.getOutputCurrent();
        inputs.voltage = indexerMotor.getBusVoltage();
    }

    @Override 
    public void setVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }


}
