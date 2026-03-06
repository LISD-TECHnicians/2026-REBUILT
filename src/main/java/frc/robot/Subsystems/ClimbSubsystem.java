package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.ClimbConstants;
import frc.robot.generated.TunerConstants;

public class ClimbSubsystem extends SubsystemBase {
    
    private TalonFX m_climbMotor;
    private VelocityVoltage m_climbVelocityRequest
        = new VelocityVoltage(0);
    private VoltageOut m_climbVoltageRequest
        = new VoltageOut(0);

    public ClimbSubsystem() {
        m_climbMotor = new TalonFX(ClimbConstants.kClimbMotorID, TunerConstants.kCANBus);
    }

    public void setClimbMotorSpeed(double setSpeed) {
        m_climbMotor.setControl(
        m_climbVoltageRequest.withOutput
        (Units.Volts.of(setSpeed * CTREConstants.kBatterySupplyVolts.in(Units.Volts))));
    }

    public void setClimbMotorSpeed(AngularVelocity setSpeed) {
        m_climbMotor.setControl(
        m_climbVelocityRequest.withVelocity
        (Units.RadiansPerSecond.of(setSpeed.in(Units.RadiansPerSecond)))); 
    }

    public void stopClimb() {
        m_climbMotor.set(0);
    }

    @Override
    public void periodic() {
        
    }
}
