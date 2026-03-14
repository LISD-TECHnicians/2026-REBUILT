package frc.robot.subsystems;

import java.util.List;

import javax.print.attribute.standard.Media;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
//import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PhysicsConstants;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AimHelper;
import frc.robot.util.ShooterHelper;

public class ShooterSubsystem extends SubsystemBase{

    //private final TalonFX m_centerShooterMotor;
    private final TalonFX m_leftShooterMotor;
    private final TalonFX m_rightShooterMotor;
    private final TalonFX m_indexerShooterMotor;
    private final List<TalonFX> m_shooterMotors;
    private final Servo m_hoodServoLH;
    private final Servo m_hoodServoRH;
    private Time m_FPGATimeStamp;
    private final VelocityVoltage m_velocityRequest;
    private final VoltageOut m_indexerVoltageRequest;
    private final VoltageOut m_shooterVoltageRequest;
    private TalonFXConfiguration m_shooterMotorConfig;
    private TalonFXConfiguration m_indexerMotorConfig;
    private final MedianFilter m_tagentialVelocityFilter;
    private final MedianFilter m_servoSetFilter;
    private double m_tangentialVelocity;
    private double m_discriminant;
    private double m_shootingDistance;
    private double m_currentServoPosition;
    private double m_targetServoPositon;
    private ShooterHelper m_helperValues;
    private Follower m_follower;
    

    public ShooterSubsystem() {
        m_FPGATimeStamp = Units.Seconds.of(0);
        //m_centerShooterMotor = new TalonFX(ShooterConstants.kCenterShooterMotorID, TunerConstants.kCANBus);
        m_leftShooterMotor = new TalonFX(ShooterConstants.kLeftShooterMotorID, TunerConstants.kCANBus);
        m_rightShooterMotor = new TalonFX(ShooterConstants.kRightShooterMotorID, TunerConstants.kCANBus);
        m_follower = new Follower(m_leftShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned).withUpdateFreqHz(100);
        m_indexerShooterMotor = new TalonFX(ShooterConstants.kShooterMotorIndexerID, TunerConstants.kCANBus);
        m_shooterMotors = List.of(m_leftShooterMotor, m_rightShooterMotor);
        m_velocityRequest = new VelocityVoltage(0);
        m_shooterVoltageRequest = new VoltageOut(0);
        m_indexerVoltageRequest = new VoltageOut(0);
        m_hoodServoLH = new Servo(ShooterConstants.kPWMChannelLH);
        m_hoodServoRH = new Servo(ShooterConstants.kPWMChannelRH);
        m_tagentialVelocityFilter = new MedianFilter(100);
        m_servoSetFilter = new MedianFilter(20);
        m_currentServoPosition = ShooterConstants.kServoInitPosition;
        m_targetServoPositon = ShooterConstants.kServoInitPosition;
        m_helperValues = new ShooterHelper(0.0, 0.0);
        
        
        configureMotors();
        m_leftShooterMotor.getMotorVoltage().setUpdateFrequency(100);
        m_leftShooterMotor.getDutyCycle().setUpdateFrequency(100);
        m_leftShooterMotor.getTorqueCurrent().setUpdateFrequency(100);
    }

    public void configureMotors() {
        m_shooterMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(ShooterConstants.kShooterInvertedValue)
                    .withNeutralMode(ShooterConstants.kShooterNeutralModeValue)
                
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(false) // testing 
                    .withSupplyCurrentLimit(Units.Amps.of(120))
                    .withSupplyCurrentLimitEnable(false)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Units.Volts.of(11))
                    .withPeakReverseVoltage(Units.Volts.of(-11))
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(.5) // .5
                .withKI(0) // 0
                .withKD(0) 
                .withKV(.15) // test this as maxRPS / 12v 1.0 --> 1 // .15
                .withKA(0.0) // --> //0.0
            );
        m_indexerMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(ShooterConstants.kShooterInvertedValue)
                    .withNeutralMode(ShooterConstants.kShooterNeutralModeValue)
                
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(false) // testing 
                    .withSupplyCurrentLimit(Units.Amps.of(120))
                    .withSupplyCurrentLimitEnable(false)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Units.Volts.of(11))
                    .withPeakReverseVoltage(Units.Volts.of(-11))
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(0.5)
                .withKI(2)
                .withKD(0)
                .withKV(0.125) // test this as maxRPS / 12v
            );
            //m_centerShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_leftShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            //m_rightShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_indexerShooterMotor.getConfigurator().apply(m_indexerMotorConfig);

        m_hoodServoLH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_hoodServoRH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    private static final InterpolatingTreeMap <Distance, ShooterHelper> m_shooterMap = 
        new InterpolatingTreeMap<> (
            (startDistance, endDistance, q) -> 
                InverseInterpolator.forDouble()
                    .inverseInterpolate(startDistance.in(Units.Meters), endDistance.in(Units.Meters), q.in(Units.Meters)),
            (startValue, endValue, t) -> 
                new ShooterHelper(
                        Interpolator.forDouble()
                            .interpolate(startValue.m_shooterWheelVelocity, endValue.m_shooterWheelVelocity, t),
                        Interpolator.forDouble()
                            .interpolate(startValue.m_hoodSetValue, endValue.m_hoodSetValue, t)
                )
        );

    static {
        /* 
         * WIP: Mapping post shooter re-design 
        */
        m_shooterMap.put(Units.Meters.of(1.9452), new ShooterHelper(275, 0.35));
        m_shooterMap.put(Units.Meters.of(2.4720), new ShooterHelper(300, 0.425));
        m_shooterMap.put(Units.Meters.of(2.8610), new ShooterHelper(300, 0.475));
        m_shooterMap.put(Units.Meters.of(3.4200), new ShooterHelper(310, 0.50));
        m_shooterMap.put(Units.Meters.of(3.9800), new ShooterHelper(330, 0.525));
        m_shooterMap.put(Units.Meters.of(4.2270), new ShooterHelper(335, 0.525));
    }

    public void energize(double distance) {
        m_helperValues = m_shooterMap.get(Units.Meters.of(distance));
        setServo(m_helperValues.m_hoodSetValue); 
        setShooterRadiansSecond(m_helperValues.m_shooterWheelVelocity);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(m_targetServoPositon, 
            m_currentServoPosition, 
            ShooterConstants.kServoPositionTolerance);
    }

    public void updateServoPosition() {
        Time currentTime = Units.Seconds.of(Timer.getFPGATimestamp());
        Time deltaTime = currentTime.minus(m_FPGATimeStamp);
        m_FPGATimeStamp = currentTime;

        if (isPositionWithinTolerance()) {
            m_currentServoPosition  = m_targetServoPositon; return; 
        }

        Distance maxServoDistance = 
            ShooterConstants.kMaxLinearServoVelocity.times(deltaTime);
        double maxServoPercentage = maxServoDistance.div(ShooterConstants.kMaxLinearServoDistance).in(Units.Value);
        m_currentServoPosition = m_targetServoPositon > m_currentServoPosition
            ? Math.min(m_targetServoPositon, m_currentServoPosition + maxServoPercentage)
            : Math.max(m_targetServoPositon, m_currentServoPosition - maxServoPercentage);        
    }

    public void setServo(double setValue) {
        m_targetServoPositon = setValue;
    }

    public void testServo(double setPosition) {
        m_hoodServoLH.setPosition(setPosition);
        m_hoodServoRH.setPosition(setPosition);
    }

    public boolean isIndividualMotorAtSpeed(/*TalonFX motor*/) {
        //System.out.println("rads/s" + (motor.getVelocity().getValueAsDouble() * 2 * Math.PI)); // confirm and set constant 
        final boolean isVelocityMode = m_leftShooterMotor.getAppliedControl().equals(m_velocityRequest);
        final AngularVelocity currentAngularVelocity = m_leftShooterMotor.getVelocity().getValue();
        final AngularVelocity targetAngularVelocity = m_velocityRequest.getVelocityMeasure();
        return isVelocityMode && currentAngularVelocity.isNear(targetAngularVelocity, ShooterConstants.kVelocityToleranceRot.times(0.5)); 
    }

    //public boolean shooterAtFireSpeed() {
    //    return m_shooterMotors.stream().allMatch(this::isIndividualMotorAtSpeed);
    //}

    public void setShooterRadiansSecond(double setSpeed) {
        m_velocityRequest.withVelocity(Units.RadiansPerSecond.of(setSpeed));
        m_leftShooterMotor.setControl(m_velocityRequest);
        // m_leftShooterMotor.setVoltage(12);//m_velocityRequest);//.setControl(m_velocityRequest);leader needs voltage status signal enabled 
        m_rightShooterMotor.setControl(m_follower.withUpdateFreqHz(100));

        // m_rightShooterMotor.set(m_leftShooterMotor.getDutyCycle().getValueAsDouble());//m_velocityRequest.withVelocity(Units.RadiansPerSecond.of(setSpeed)));


        System.out.println(m_leftShooterMotor.getVelocity().getValueAsDouble() * 2 * Math.PI);
        // System.out.println("DIFF" + (m_rightShooterMotor.getVelocity().getValueAsDouble() - m_leftShooterMotor.getVelocity().getValueAsDouble()));

        /*{
            for (TalonFX shooterMotor : m_shooterMotors) {
            shooterMotor.setControl(
                m_velocityRequest
                    .withVelocity(Units.RadiansPerSecond.of(setSpeed))
            );
        }*/
    }


    public void setIndexerMotorPercentage(double setSpeed) {
        m_indexerShooterMotor.setControl(m_shooterVoltageRequest
            .withOutput(CTREConstants.kBatterySupplyVolts.times(setSpeed)));
    }

     public void setShooterPercentage(double setSpeed) {
        for (final TalonFX shooterMotor : m_shooterMotors) {
            shooterMotor.setControl(
                m_shooterVoltageRequest
                    .withOutput(CTREConstants.kBatterySupplyVolts.times(setSpeed))
            );
        }
     }

    public void stopShooterMotors() {
        setShooterPercentage(0);
    }

    public void stopIndexerMotor() {
        setIndexerMotorPercentage(0.0);
    }

    public void reverseIndexerMotor(double setSpeed) {
        m_indexerShooterMotor.setControl(m_shooterVoltageRequest
            .withOutput(CTREConstants.kBatterySupplyVolts.times(-setSpeed)));
    }

    public double getTangentialVelocity() {
        double wheelRPM = m_leftShooterMotor.getVelocity().getValueAsDouble() * 60.0; // Convert rot/s to RPM
        double radiansPerSecond = (wheelRPM * 2 * Math.PI) / 60.0;
        return radiansPerSecond * ShooterConstants.kRadiusShooterWheel;
    }

    @Override
    public void periodic() { 
        updateServoPosition();
        //m_hoodServoLH.set(m_targetServoPositon + ShooterConstants.kLeftServoOffset);
        //m_hoodServoRH.set(m_targetServoPositon);

        if (getCurrentCommand() == null) {
            setShooterPercentage(ShooterConstants.kIdleShooterPercentage); 
            setServo(ShooterConstants.kServoInitPosition);
        }
    }
}


