package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {

    private double m_tangentialVelocity;
    private double m_theta; 
    private double m_discriminant;
    private double m_shootingDistance;
    private SparkMax m_hoodMotor;
    private SparkMax m_shooterMotor;
    private SparkMaxConfig m_hoodMotorConfig;
    private SparkMaxConfig m_shooterMotorConfig;
    private RelativeEncoder m_hoodEncoder;
    private RelativeEncoder m_shooterEncoder;
    private ProfiledPIDController m_pidControllerHood;
    private Constraints m_hoodTrapezoidConstraints;

    public ShooterSubsystem() {

        m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorID, MotorType.kBrushless);
        m_hoodMotorConfig = new SparkMaxConfig();

        m_hoodMotorConfig.idleMode(IdleMode.kBrake)
                          .inverted(false);

        m_hoodMotor.configure(m_hoodMotorConfig, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kNoPersistParameters);

        m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorID, 
                                       MotorType.kBrushless);

        m_shooterMotorConfig = new SparkMaxConfig();

        m_shooterMotorConfig.idleMode(IdleMode.kCoast)
                            .inverted(false);

        m_shooterMotor.configure(m_shooterMotorConfig, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kNoPersistParameters);

        m_hoodEncoder = m_hoodMotor.getEncoder();
        m_shooterEncoder = m_shooterMotor.getEncoder();

        m_hoodTrapezoidConstraints = new Constraints(ShooterConstants.kHoodDegreesSecond,
                                          ShooterConstants.kHoodDegreesSecondSquare);

        m_pidControllerHood = new ProfiledPIDController(ShooterConstants.kHoodProportion,
                                                 ShooterConstants.kHoodIntegration,
                                                 ShooterConstants.kHoodDerivative,
                                                 m_hoodTrapezoidConstraints);

        m_pidControllerHood.setTolerance(ShooterConstants.kHoodPositionVarience);
    }

    public double getCalculatedPitch() 
    {
        m_shootingDistance = getShootingDistance(); 
        m_shootingDistance = (m_shootingDistance == 0) ? 0.1 : m_shootingDistance; 
        
        m_tangentialVelocity = getTangentialVelocity() * ShooterConstants.kEffectiveKineticCoef;
        
        m_discriminant = Math.pow(m_tangentialVelocity, 4) - PhysicsConstants.kGravity
             * (PhysicsConstants.kGravity * Math.pow(m_shootingDistance, 2) + 2 * 
                Math.pow(m_tangentialVelocity, 2) * PhysicsConstants.kDeltaHeight);
        
        if (m_discriminant < 0) {
            return 0.0; // Target unreachable 
        }
        
        double tanTheta1 = (Math.pow(m_tangentialVelocity, 2) - Math.sqrt(m_discriminant)) 
                / (PhysicsConstants.kGravity * m_shootingDistance); 

        double tanTheta2 = (Math.pow(m_tangentialVelocity, 2) + Math.sqrt(m_discriminant)) 
            / (PhysicsConstants.kGravity * m_shootingDistance); // Higher Arc 

        // Need to clamp this based on limitations of the bot! 
        m_theta = Math.atan(tanTheta2);
        m_theta = Math.toDegrees(m_theta);
        
        return m_theta; 
    }

    public double getHoodPitch()
    {
        double hoodDegrees = m_hoodEncoder.getPosition() * 360 * ShooterConstants.kHoodGearRatio;
        double convertedDegrees = hoodDegrees + (ShooterConstants.kHoodPlaneCorrection 
                                               +  ShooterConstants.kHoodCorrection) 
                                               -  ShooterConstants.kHoodRestAngle;     
        return convertedDegrees; 
    }

    public State getHoodGoal() {
        return m_pidControllerHood.getGoal();
    }

    public boolean getHoodAtGoal() {
        return m_pidControllerHood.atGoal();
    }

    public void setHoodMotorSpeed(double setSpeed) {
        m_hoodMotor.set(MathUtil.clamp(setSpeed, 
                        RobotConstants.kNeoMotorSetMin, 
                        RobotConstants.kNeoMotorSetMax));
    }

    public void setHoodGoal(State goalState) {
        m_pidControllerHood.setGoal(goalState);
    }

    public void stopHoodMotor() {
        m_hoodMotor.set(0.0);
    }

    public void setShooterWheelSpeed(double setSpeed)
    {
        m_shooterMotor.set(MathUtil.clamp(setSpeed, 
                        RobotConstants.kNeoMotorSetMin, 
                        RobotConstants.kNeoMotorSetMax));
    }

    public double getShooterWheelRpm()
    {
        return m_shooterEncoder.getVelocity(); // rpm
    }

    public double getTangentialVelocity()
    {
        return ((getShooterWheelRpm() * ShooterConstants.kRadiusShooterWheel * Math.PI * 2) / 60.0); 
        // m/s --> rpm * 2pi * r / 60
    }

    public double getShootingDistance() 
    {
        return 0.0; // Complete with new SwerveSubsystem getter once available.
    }

    public boolean shooterAtSpeed() {
        return ((Math.abs(getTangentialVelocity() - ShooterConstants.kWheelSpeedTangential) 
                / ShooterConstants.kWheelSpeedTangential) <= ShooterConstants.kAllowedShooterSpeedError);
    }

    public boolean shooterFireReady() {
        return shooterAtSpeed() && getHoodAtGoal();
    }

    @Override
    public void periodic() { 
        if (getCurrentCommand() == null) {
            setShooterWheelSpeed(ShooterConstants.kWheelSpeedIdlePercent); 
        } 
        
        if (!getHoodAtGoal()) {
            setHoodMotorSpeed(MathUtil.clamp(
                         m_pidControllerHood.calculate(getHoodPitch(), 
                         m_pidControllerHood.getGoal()), 
                         RobotConstants.kNeoMotorSetMin, 
                         RobotConstants.kNeoMotorSetMax));
        }
        else {
            stopHoodMotor();
        }
    } 
}


