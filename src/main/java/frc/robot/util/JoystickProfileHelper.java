package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class JoystickProfileHelper {
    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;
    private final DoubleSupplier rotation;

    private static final double joystickDeadband = .10; // 10% deadband 
    private static final double joystickExponential = 1.5; // used for exponential gain when mapping input over valid region.

    public JoystickProfileHelper(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotation) {
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotation = rotation; 
    }

    public JoystickProfileHelper(DoubleSupplier xInput, DoubleSupplier yInput) {
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotation = () -> 0; // supplier to 0 assuming no rotational component of movement. 
    }

    public ManualDriveInput getSmoothedInput() { 
        final Vector<N2> rawTranslationInput = VecBuilder.fill(yInput.getAsDouble(), xInput.getAsDouble());
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, joystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, joystickExponential);

        final double rawRotationInput = rotation.getAsDouble();
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, joystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, joystickExponential);

        return new ManualDriveInput(
            curvedTranslationInput.get(0), 
            curvedTranslationInput.get(1), 
            curvedRotationInput
        );
    }    
}
