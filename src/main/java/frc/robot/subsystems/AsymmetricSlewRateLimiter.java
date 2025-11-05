package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

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

  public double calculate(double input) {
    double now = Timer.getFPGATimestamp();
    double deltaTime = Math.max(1e-6, now - lastTime);
    lastTime = now;
  
    double cur = lastValue;
  
    // Magnitudes and signs
    double curMag = Math.abs(cur);
    double tgtMag = Math.abs(input);
    double curSign = (cur == 0.0) ? 0.0 : Math.signum(cur);
    double tgtSign = (input == 0.0) ? 0.0 : Math.signum(input);
  
    // If sign differs (and both non-zero), that is a direction reversal -> treat as decel (braking).
    final boolean signReversal = (cur != 0.0 && input != 0.0 && curSign != tgtSign);
  
    // Choose rate:
    double rate;
    if (signReversal) {
      rate = decelRate;
    } else {
      // If target magnitude is larger -> accelerating; else decelerating
      rate = (tgtMag > curMag) ? accelRate : decelRate;
    }
  
    double maxDelta = rate * deltaTime;
  
    // Move the magnitude toward the target magnitude (limited by maxDelta)
    double magDelta = tgtMag - curMag;
    double magChange = Math.copySign(Math.min(Math.abs(magDelta), maxDelta), magDelta);
    double newMag = curMag + magChange;
  
    // Decide the sign of the new value
    double newSign;
    if (newMag == 0.0) {
      // If we've reached zero, adopt the target sign so we can accelerate on that side next tick
      newSign = tgtSign;
    } else if (curMag == 0.0 && tgtMag > 0.0) {
      // Starting from zero: go toward target sign (accelerating)
      newSign = tgtSign;
    } else {
      // If magnitude is increasing, it should adopt target sign; if decreasing, keep current sign.
      newSign = (tgtMag > curMag) ? tgtSign : curSign;
    }
  
    lastValue = Math.copySign(newMag, newSign);
    return lastValue;
  }
  
}
