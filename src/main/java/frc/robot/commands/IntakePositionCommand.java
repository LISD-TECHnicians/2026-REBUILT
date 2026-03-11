package frc.robot.commands;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;


public class IntakePositionCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;
    private Position m_targetPosition;
    private AngularVelocity m_velocity;
    private AngularAcceleration m_acceleration;
    private double m_jerk;

    public IntakePositionCommand(IntakeSubsystem intakeSubsystem, Position targetPosition) {
        m_intakeSubsystem = intakeSubsystem;
        m_targetPosition = targetPosition;
        // addRequirements(m_intakeSubsystem);
        
        configureMotionParameters();
    }

    private void configureMotionParameters() {
        // dynamic motion magic diff per position.
        switch (m_targetPosition) {
            case DEPLOYED:
                m_velocity = IntakeConstants.kPivotRunVelocity;
                m_acceleration = IntakeConstants.kPivotRunAcceleration;
                m_jerk = IntakeConstants.kPivotDeployJerk;
                break;
                
            case HOME:
                m_velocity = IntakeConstants.kPivotRunVelocity.times
                    (2.0); // Was 1.0
                m_acceleration = IntakeConstants.kPivotRunAcceleration.times
                    (2.0); // Was 1.0
                m_jerk = IntakeConstants.kPivotHomeJerk;
                break;
                
            case INDEXING:
                m_velocity = IntakeConstants.kPivotRunVelocity.times
                (2.0); // Was 15.0 
                m_acceleration = IntakeConstants.kPivotRunAcceleration.times
                (2.0); // Was 15.0
                m_jerk = IntakeConstants.kPivotIndexingJerk * 1;  // originally multiplied by 1500
                break;
                
            default:
                m_velocity = IntakeConstants.kPivotRunVelocity;
                m_acceleration = IntakeConstants.kPivotRunAcceleration;
                m_jerk = IntakeConstants.kPivotDeployJerk;
                break;
        }
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_intakeSubsystem.setPivotPosition(
            m_targetPosition,
            m_velocity,
            m_acceleration,
            m_jerk 
         );
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return m_intakeSubsystem.pivotInPosition();
        // test and consider backup plan 
    }
}
