package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ShootCommand extends Command {
    private ShooterSubsystem m_shooterSubsystem;

    public ShootCommand(ShooterSubsystem subsystem) {
        this.m_shooterSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override 
    public void initialize() {
        m_shooterSubsystem.setHoodGoal(new State(
            m_shooterSubsystem.getCalculatedPitch(), 0.0));

        m_shooterSubsystem.setShooterWheelSpeed(
            ShooterConstants.kWheelSpeedFirePercent);
    }

    @Override
    public void execute() {
      if (m_shooterSubsystem.shooterFireReady()) 
      {
        // index balls to shooter wheels
      };
    }

    @Override
    public void end(boolean interrupted) {
        // stop the indexer motors
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
