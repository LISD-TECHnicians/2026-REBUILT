package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeRunCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;

    public IntakeRunCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (m_intakeSubsystem.pivotInPosition()) {
            m_intakeSubsystem.setIntakeMotorSpeed(IntakeConstants.kIntakeSpeedRunCoef);
        } else {
            m_intakeSubsystem.stopIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}

