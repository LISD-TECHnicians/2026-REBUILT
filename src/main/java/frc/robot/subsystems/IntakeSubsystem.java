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

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakePositions {
        STORED(new State(IntakeConstants.kPivotAngleStored, 0.0)),
        ACTIVE(new State(IntakeConstants.kPivotAngleActive, 0.0)),
        OUTTAKE(new State(IntakeConstants.kPivotAngleOut, 0.0));

        private final State state;

        private IntakePositions(State state) {
            this.state = state;
        }

        public State getState() {
            return state;
        }

        public double getDegrees() {
            return state.position;
        }

        public double getVelocity() {
            return state.velocity;
        }
    }

    private SparkMax m_intakeMotor, m_pivotMotor;
    private SparkMaxConfig m_intakeMotorConfig, m_pivotMotorConfig;
    private RelativeEncoder m_intakeEncoder, m_pivotEncoder;
    private ProfiledPIDController m_pidControllerPivot;
    private Constraints m_pivotTrapezoidConstaints;

    public IntakeSubsystem() {

        m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_pivotMotor = new SparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
        
        m_intakeMotorConfig = new SparkMaxConfig();
        m_intakeMotorConfig.idleMode(IdleMode.kCoast)
                           .inverted(false); 

        m_intakeMotor.configure(m_intakeMotorConfig, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kNoPersistParameters);

        m_pivotMotorConfig = new SparkMaxConfig();
        m_pivotMotorConfig.idleMode(IdleMode.kBrake)
                          .inverted(false);

        m_pivotMotor.configure(m_pivotMotorConfig, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kNoPersistParameters);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_pivotEncoder = m_pivotMotor.getEncoder();

        m_pivotTrapezoidConstaints = new Constraints(IntakeConstants.kPivotDegreesSecond, IntakeConstants.kPivotDegreesSecondSquare);

        m_pidControllerPivot = new ProfiledPIDController(IntakeConstants.kPivotProportion, 
            IntakeConstants.kPivotIntegration, IntakeConstants.kPivotDerivative, m_pivotTrapezoidConstaints);

        m_pidControllerPivot.setTolerance(IntakeConstants.kPivotPositionVarience, IntakeConstants.kDegreesSecondVariation); 
    }

    public double getPivotPosition() {
        double rawDegrees = m_pivotEncoder.getPosition() * 360 * IntakeConstants.kPivotGearRatio;
        double convertedDegrees = 0.0; // TODO: Base calculations off of mechanical design once built
        return convertedDegrees;
    }

    public double getIntakeSpeed() {
        double intakeRPM = m_intakeEncoder.getVelocity();
        return (intakeRPM / RobotConstants.kNeoMaxRpm) * 100; // % value for pid
    }

    public void setPivotState(State goalState) {
        m_pidControllerPivot.setGoal(goalState);
    }

    public void setPivotSpeed(double setSpeed) {
        m_pivotMotor.set(setSpeed);
    }

    public void stopPivotMotor() {
        m_pivotMotor.set(0.0);
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        m_intakeMotor.set(0); 
    }

    public boolean atPosition() {
        return m_pidControllerPivot.atGoal();
    }

    @Override
    public void periodic() {
         if (!atPosition()) {
        setPivotSpeed(MathUtil.clamp(m_pidControllerPivot.calculate(getPivotPosition(),
                      m_pidControllerPivot.getGoal()), 
                      RobotConstants.kNeoMotorSetMin, RobotConstants.kNeoMotorSetMax));
        }
        else {
            m_pivotMotor.stopMotor();
        }
    }
}
