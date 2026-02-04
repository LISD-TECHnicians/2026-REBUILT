package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.RobotConstants.IndexerConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class IndexerSubsystem extends SubsystemBase{
    
    private TalonFX m_indexerMotor;   
    private VelocityVoltage m_indexerVelocityRequest 
        = new VelocityVoltage(0);
    
    public IndexerSubsystem() {
        m_indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorID, TunerConstants.kCANBus);
    }

    public void setIndexerMotorSpeed(double setSpeed) {
        m_indexerMotor.setVoltage(setSpeed * CTREConstants.kBatterySupplyVolts.in(Units.Volts));
    }

    public void setIndexerMotorSpeed(AngularVelocity setSpeed) {
        m_indexerMotor.setControl(
        m_indexerVelocityRequest.withVelocity
        (Units.RadiansPerSecond.of(setSpeed.in(Units.RadiansPerSecond))));
    }

    public void stopIndexer() {
        m_indexerMotor.set(0);
    }

    @Override
    public void periodic() {
        // TODO: Add logging with smartdashboard 
    }
}