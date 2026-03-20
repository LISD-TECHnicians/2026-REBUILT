package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs; 
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
// TODO: Test and refine pivot positions
    public enum Position {
        DEPLOYED(-14.5), //-14.65 PREV
        INDEXING(-9.41),
        HOME(-0.91);

        private double rotations;

        private Position(double rotations) {
            this.rotations = rotations;
        }

        public Angle positionRotations() {
            return Units.Rotations.of(this.rotations);
        }
    }

    private TalonFX m_intakeMotor;
    public TalonFX m_pivotMotor;    
    private VelocityVoltage m_intakeVelocityRequest 
        = new VelocityVoltage(0);
    private VoltageOut m_intakeVoltageRequest 
        = new VoltageOut(0);
    
    /*
    private DynamicMotionMagicVoltage m_pivotDynamicMotionMagicRequest 
            = new DynamicMotionMagicVoltage(
            Units.Rotations.of(0),  
            IntakeConstants.kPivotRunVelocity, 
            IntakeConstants.kPivotRunAcceleration);
    */

    private VoltageOut m_pivotVoltageRequest
        = new VoltageOut(0); 

    DynamicMotionMagicVoltage m_pivotDynamicMotionMagicRequest
            = new DynamicMotionMagicVoltage(
                Units.Rotations.of(1),  
                IntakeConstants.kPivotRunVelocity, 
                IntakeConstants.kPivotRunAcceleration);

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, TunerConstants.kCANBus.getName());
        m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID, TunerConstants.kCANBus.getName());
        configureMotors();
    }

    public void configureIntakeMotor() {
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kIntakeInvertedValue)
                    .withNeutralMode(IntakeConstants.kIntakeNeutralModeValue)
                
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(false)
                    .withSupplyCurrentLimit(Units.Amps.of(70))
                    .withSupplyCurrentLimitEnable(false)
            );
            m_intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    public void configurePivotMotor() {
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kPivotInvertedValue)
                    .withNeutralMode(IntakeConstants.kPivotNeutralModeValue)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Units.Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(-.35)
                .withReverseSoftLimitThreshold(-14.65)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(IntakeConstants.kPivotMotorGearReduction)
            ) 
            .withSlot0(
                new Slot0Configs()
                    .withKP(1.5)//IntakeConstants.kPivotSlot0KP) // 1.5 Originally
                    .withKI(IntakeConstants.kPivotSlot0KI)
                    .withKD(IntakeConstants.kPivotSlot0KD)
                    .withKV(0.2/*CTREConstants.kBatterySupplyVolts.in(Units.Volts) // 0.2 Originally
                    / (CTREConstants.kKrakenX60MaxRPS.in(Units.RotationsPerSecond)
                    * (IntakeConstants.kPivotMotorGearReduction)*/)
                    .withKG(1) // 1 Originally
                    .withKA(.1) // .1 Originally
            );
            m_pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }       

    public void configureMotors() {
        configureIntakeMotor();
        configurePivotMotor();
    }
 
    public void setPivotPosition(Position position,
        AngularVelocity requestVelocity, 
        AngularAcceleration requestAcceleration,
        double requestJerk) {
        
        Angle targetRotations = position.positionRotations();
        //Angle targetRotations = Units.Rotations.of(targetDegrees.in(Units.Degrees) / 360.0);
        
        m_pivotDynamicMotionMagicRequest
            .withPosition(targetRotations)
            .withVelocity(requestVelocity)
            .withAcceleration(requestAcceleration)
            .withJerk(requestJerk);
        

        m_pivotMotor.setControl(m_pivotDynamicMotionMagicRequest);
    }

    public void setIntakeMotorSpeed(double setSpeed) {
        m_intakeMotor.setControl(
            m_intakeVoltageRequest
                .withOutput(Units.Volts.of(
                    setSpeed
                    * CTREConstants.kBatterySupplyVolts.in(Units.Volts)
                    )
                )
        );
    }

    public void setPivotMotorSpeed(double setPivotSpeed) {
        m_pivotMotor.setControl(
            m_pivotVoltageRequest
                .withOutput(Units.Volts.of(
                    setPivotSpeed
                    * CTREConstants.kBatterySupplyVolts.in(Units.Volts)
                    )
                )
        );
    } 

    public void setIntakeMotorSpeed(AngularVelocity setSpeed) {
        m_intakeMotor.setControl(
            m_intakeVelocityRequest
                .withVelocity(Units.RadiansPerSecond.of(
                    setSpeed.in(Units.RadiansPerSecond)
                    )
                )
        );
    }
 
    public boolean pivotInPosition() {
        final Angle currentPosition = m_pivotMotor.getPosition().getValue();
        final Angle targetPosition = m_pivotDynamicMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(Units.Rotations.of(targetPosition.in(Units.Rotations)), 
            IntakeConstants.kPivotRotationsTolerance);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void stopPivot() {
        m_pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        // Testing Pivot Positions --. HOME @ -.35 Rotations, Deployed @ -14.65 Rotations, Ocillate @ -8.75 Rotations
        SmartDashboard.putNumber("Intake Position:", m_pivotMotor.getPosition().getValue().in(Units.Rotations));
    }
}
