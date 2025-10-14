package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

/**
 * Smooths a changing input over time using exponential smoothing.
 */
public class DriveIntervalGradient {

    private final DoubleSupplier input; // source value (e.g., joystick)
    private final double alpha;         // smoothing factor (0 < alpha <= 1)
    private double lastValue;           // last smoothed value

    /**
     * @param input  Supplier of the raw value
     * @param alpha  Fraction of difference applied per update (0.0 = no change, 1.0 = instant)
     */
    public DriveIntervalGradient(DoubleSupplier input, double alpha) {
        this.input = input;
        this.alpha = alpha;
        this.lastValue = input.getAsDouble(); // initialize to current input
    }

    /** Call this once per control loop to update the smoothed value. */
    public void update() {
        lastValue += (input.getAsDouble() - lastValue) * alpha;
    }

    /** Returns the current smoothed value. */
    public double getValue() {
        return lastValue;
    }

    /** Returns a DoubleSupplier for convenience in commands. */
    public DoubleSupplier getValueSupplier() {
        return this::getValue;
    }
}
