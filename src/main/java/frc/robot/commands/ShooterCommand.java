package frc.robot.commands;


import edu.wpi.first.units.Units;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.util.AimHelper;

public class ShooterCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private CommandSwerveDrivetrain m_driveSubsystem;
    private boolean m_shooterReadyFire = false;
    private MedianFilter m_distanceFilter;

    public ShooterCommand (ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain driveSubsystem){
        m_shooterSubsystem = shooterSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_distanceFilter = new MedianFilter(10);
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

       
       double shootingDistance = AimHelper.getHubPosition().getDistance(m_driveSubsystem.getState().Pose.getTranslation());
       shootingDistance = m_distanceFilter.calculate(shootingDistance);
       m_shooterSubsystem.energize(shootingDistance);
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
