package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends Command{
    private FeederSubsystem m_feederSubsystem;
    private final double setSpeed;

    public FeederCommand (FeederSubsystem feederSubsystem, double speed) {
        m_feederSubsystem = feederSubsystem;
        setSpeed = speed;
        addRequirements(m_feederSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       m_feederSubsystem.setIndexerMotorSpeed(setSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}

