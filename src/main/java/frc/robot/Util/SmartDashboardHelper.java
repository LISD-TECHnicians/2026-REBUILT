package frc.robot.Util;

import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.units.Units;

public class SmartDashboardHelper {
private final double totalTimeTeleSeconds = 140.0;
private final double totalTimeAutoSeconds = 20.0;
private final double totalTimeSeconds 
    = totalTimeAutoSeconds + totalTimeTeleSeconds;
private final int phaseDuration = 25;

public SmartDashboardHelper() {}

private int currentPhase = 0;

private boolean blueAllianceActiveFirst = false; 

private final Optional<Alliance> alliance = DriverStation.getAlliance();

private String currentPeriod() {return (DriverStation.isTeleopEnabled()) ? "teleop" : "auto";}

private double periodTotalSeconds() {return (currentPeriod() == "teleop") ? totalTimeTeleSeconds : totalTimeAutoSeconds;}

private boolean matchStarted() {return !alliance.isEmpty();}

private String gameData() {return DriverStation.getGameSpecificMessage();}

private String firstActiveAlliance() {return (gameData().charAt(0) == 'B') ? "Blue" : "Red";}

private void setCurrentPhase(int phase) {currentPhase = phase;} 


// Accessors here 
    public boolean getIsActivePhase() {
        if (matchStarted()) {return false;}
        if (DriverStation.isAutonomousEnabled()) {return true;}
        else if (!DriverStation.isTeleopEnabled()) {return false;} // layover period of 10 seconds
        double matchPeriodTime = DriverStation.getMatchTime();
        
        if (gameData().isEmpty()) {return true;} // ensuring that gameData returns a string before we attempt to get a character from it.
        if(firstActiveAlliance() == "Blue") {blueAllianceActiveFirst = true;}

        boolean phase1ActiveState = true;
        switch (alliance.get()) {
            case Red:
                  phase1ActiveState = !blueAllianceActiveFirst;
                 break;
            case Blue:
                 phase1ActiveState = blueAllianceActiveFirst; 
                break;
            default:
                 phase1ActiveState = blueAllianceActiveFirst;
                break;
        };

        if (matchPeriodTime > 130) {return true;}
        else if (matchPeriodTime > 105) {setCurrentPhase(1); return phase1ActiveState;}
        else if (matchPeriodTime > 80) {setCurrentPhase(2); return !phase1ActiveState;} 
        else if (matchPeriodTime > 55) {setCurrentPhase(3); return phase1ActiveState;} 
        else if (matchPeriodTime > 30) {setCurrentPhase(4); return !phase1ActiveState;} 
        else {return true;}
    }

    public String getHubMessage() {
    if (getIsActivePhase()) return "ACTIVE HUB";
    return "INACTIVE HUB";
    }

    public int getCurrentPhase() {return currentPhase;}

    public double getMatchTimeRemaining() { // for whole match over both periods. 
        if (!matchStarted()) {return totalTimeSeconds;}
        return (totalTimeSeconds - (periodTotalSeconds() - getReaminingPeriodTime())); 
    }

    public double getReaminingPeriodTime() {return DriverStation.getMatchTime();} 

    //private static double phaseTimeRemaining() {return matchPhaseSeconds[matchPhaseIndx] - DriverStation.getMatchTime();} // correct this difference 
}



