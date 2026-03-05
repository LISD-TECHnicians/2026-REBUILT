package frc.robot.commands;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;


public class IntakeOscillateCommand extends Command {

    private IntakeSubsystem m_intakeSubsystem;
    private Position m_currentTargetPosition;
    private boolean m_hasReachedPosition;

    public IntakeOscillateCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

        /* 
        m_velocity = IntakeConstants.kPivotRunVelocity.times(
                        IntakeConstants.kPivotOscillateVelocityCoef);
        m_acceleration = IntakeConstants.kPivotRunAcceleration.times(
                    IntakeConstants.kPivotOscillateAccelerationCoef);
        m_jerk = IntakeConstants.kPivotOscillateJerk;
        */
    }

    @Override 
    public void initialize() {}

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
            m_velocity,
            m_acceleration,
            m_jerk
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
