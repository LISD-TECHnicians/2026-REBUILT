package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.RobotConstants.ClimbConstants;
import frc.robot.Constants.RobotConstants.FeederConstants;

public class ClimbCommand extends Command{
    private ClimbSubsystem m_climbSubsystem;
    private final double setSpeed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double speed) {
        m_climbSubsystem = climbSubsystem;
        setSpeed = speed;
        addRequirements(m_climbSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       m_climbSubsystem.setClimbMotorSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbSubsystem.stopClimb();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
