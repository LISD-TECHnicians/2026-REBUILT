package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command{
    private IndexerSubsystem m_indexerSubsystem;
    private final double setSpeed;

    public IndexerCommand (IndexerSubsystem indexerSubsystem, double speed) {
        m_indexerSubsystem = indexerSubsystem;
        setSpeed = speed;
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       m_indexerSubsystem.setIndexerMotorSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}

