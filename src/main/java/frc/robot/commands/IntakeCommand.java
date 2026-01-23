package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePositions;

public class IntakeCommand extends Command {
    
    public enum IntakeMode {
        INTAKE,   
        OUT,  
        STORE     
    }
    
    private IntakeSubsystem m_intakeSubsystem;
    private IntakeMode m_mode;

    public IntakeCommand(IntakeSubsystem subsystem, IntakeMode mode) {
        this.m_intakeSubsystem = subsystem;
        this.m_mode = mode;
        addRequirements(subsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        switch (m_mode) {
            case INTAKE:
                m_intakeSubsystem.setIntakeSpeed(IntakeConstants.kActiveIntakeSpeed); 
                m_intakeSubsystem.setPivotState(IntakePositions.ACTIVE.getState());
                break;
            case OUT:
                m_intakeSubsystem.setIntakeSpeed(-IntakeConstants.kActiveIntakeSpeed); 
                m_intakeSubsystem.setPivotState(IntakePositions.OUTTAKE.getState());
                break;
            case STORE:
                m_intakeSubsystem.setPivotState(IntakePositions.STORED.getState());
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
