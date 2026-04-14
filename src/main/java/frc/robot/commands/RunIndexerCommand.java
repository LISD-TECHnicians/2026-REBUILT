package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunIndexerCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private final double setSpeed;
    
    public RunIndexerCommand (ShooterSubsystem shooterSubsystem, double speed) {
        m_shooterSubsystem = shooterSubsystem;
        setSpeed = speed;
    }
    
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooterSubsystem.setIndexerMotorPercentage(setSpeed);
        // m_shooterSubsystem.setShooterPercentage(setSpeed);
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
