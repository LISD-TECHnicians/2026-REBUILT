package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class IndexerCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;

    public IndexerCommand (ShooterSubsystem shooterSubsystem){
        m_shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
       m_shooterSubsystem.setIndexerMotorPercentage(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopIndexerMotor();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
