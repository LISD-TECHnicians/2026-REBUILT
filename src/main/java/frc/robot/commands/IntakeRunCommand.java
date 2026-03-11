package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRunCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;
    private final double setSpeed;


    public IntakeRunCommand(IntakeSubsystem intakeSubsystem, double speed) {
        m_intakeSubsystem = intakeSubsystem;
        setSpeed = speed;
        //addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
       m_intakeSubsystem.setIntakeMotorSpeed(setSpeed);
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

