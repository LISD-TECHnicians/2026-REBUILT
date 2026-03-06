package frc.robot.util;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterHelper {
    public double m_shooterWheelVelocity;
    public double m_hoodSetValue;

    public ShooterHelper(double wheelVelocity, double setValue)
    {
        m_shooterWheelVelocity = wheelVelocity;
        m_hoodSetValue = setValue;
    }
}