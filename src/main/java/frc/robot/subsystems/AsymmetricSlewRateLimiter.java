package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

/**
 * Based on the FRC provided SlewRaterLimiter class, but this allows the decelration to be managed separately from 
 * the acceleration. 'Slew' is an industry term for 'rate of change'. This class is useful for drivetrains where
 * you want to be accelerate over time, but slow down faster than you speed up (asymmetric).
 * 
 * accelRate = maximum rise rate (units per second)
 * decelRate = maximum fall rate (units per second) — set higher to brake faster
 */
public class AsymmetricSlewRateLimiter {
  private final double accelRate;
  private final double decelRate;

  private double lastValue = 0.0;
  private double lastTime = Timer.getFPGATimestamp();

  public AsymmetricSlewRateLimiter(double accelRate, double decelRate) {
    if (accelRate < 0 || decelRate < 0) {
      throw new IllegalArgumentException("Rates must be non-negative");
    }
    this.accelRate = accelRate;
    this.decelRate = decelRate;
  }

  /** 
   * Used to calculate the new value based on the input and the slew rate limits. It will compare
   * the last calculation time and value to determine how much to change the output.
   */
  public double calculate(double input) {
    double now = Timer.getFPGATimestamp();

    // prevent divide by zero, value should be around .02s given how often the periodic methods are executed. 
    double deltaTime = Math.max(1e-6, now - lastTime); 

    // update state for the next calculation
    lastTime = now;

    // Compute the maximum allowed change this tick based on elapsed time and slew rates
    double deltaInput = input - lastValue;
    double maxDelta = deltaTime * ((deltaInput > 0.0) ? accelRate : decelRate);

    //update input value with new limited value
    lastValue += signum(deltaInput) * Math.min(abs(deltaInput), maxDelta);

    return lastValue;
  }
}
