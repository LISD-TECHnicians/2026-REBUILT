package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.RobotConstants.PhysicsConstants;

public class ShooterCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private boolean m_shooterReadyFire = false;

    public ShooterCommand (ShooterSubsystem shooterSubsystem){
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TODO: test, refine values and set to using si units -> rads/s
       m_shooterSubsystem.setShooterRadiansSecond(400); //575
       m_shooterReadyFire = m_shooterSubsystem.shooterAtFireSpeed();
       if (m_shooterReadyFire) {m_shooterSubsystem.setIndexerMotorPercentage(1.0);}
       else {m_shooterSubsystem.setIndexerMotorPercentage(0.0);}
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopShooterMotors();
        m_shooterSubsystem.stopIndexerMotor();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
