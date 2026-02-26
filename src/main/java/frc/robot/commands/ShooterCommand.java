package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.RobotConstants.PhysicsConstants;

public class ShooterCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;

    public ShooterCommand (ShooterSubsystem shooterSubsystem){
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

     @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       m_shooterSubsystem.setShooterPercentage(0.7);
       m_shooterSubsystem.setIndexerMotorPercentage(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopShooterMotors();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
