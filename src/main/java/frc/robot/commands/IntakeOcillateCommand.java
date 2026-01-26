package frc.robot.commands;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;


public class IntakeOcillateCommand extends Command{

    private IntakeSubsystem m_intakeSubsystem;
    private Position m_currentTargetPosition;
    private AngularVelocity m_velocity;
    private AngularAcceleration m_acceleration;
    private double m_jerk;
    private boolean m_hasReachedPosition;

    public IntakeOcillateCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        
        configureMotionParameters();
    }

    private void configureMotionParameters() {
        m_velocity = IntakeConstants.kPivotRunVelocity;
        m_acceleration = IntakeConstants.kPivotRunAcceleration;
        m_jerk = IntakeConstants.kPivotDeployJerk;
    }
    
    @Override 
    public void initialize() {
        // assuming deployed will be first
        m_currentTargetPosition = Position.DEPLOYED;
        m_hasReachedPosition = false;
    
        m_intakeSubsystem.setPivotPosition(
            m_currentTargetPosition,
            m_velocity,
            m_acceleration,
            m_jerk
        );
    }

    @Override 
    public void execute() {
        if (m_intakeSubsystem.pivotInPosition()) {
           
            if (!m_hasReachedPosition) {
                m_hasReachedPosition = true;
              
                if (m_currentTargetPosition == Position.DEPLOYED) {
                    m_currentTargetPosition = Position.INDEXING;
                } else {
                    m_currentTargetPosition = Position.DEPLOYED;
                }
                
                m_intakeSubsystem.setPivotPosition(
                    m_currentTargetPosition,
                    m_velocity,
                    m_acceleration,
                    m_jerk
                );
            }
        } 
        else {
            m_hasReachedPosition = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setPivotPosition(
            Position.DEPLOYED,
            IntakeConstants.kPivotRunVelocity.times(IntakeConstants.kPivotDeployVelocityCoef),
            IntakeConstants.kPivotRunAcceleration.times(IntakeConstants.kPivotDeployAccelerationCoef),
            IntakeConstants.kPivotDeployJerk
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
