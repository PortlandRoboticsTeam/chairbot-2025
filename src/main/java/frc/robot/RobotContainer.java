package frc.robot;

import frc.robot.subsystems.DriveIntervalGradient;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  private static final double DEADBAND = 0.1;
  
  /**
   * Fraction of joystick input applied per control loop to determine how quickly to ramp up to full speed.
   * 
   * Calculation:
   *   alpha = 1 - (0.01)^(loopTime / desiredTimeToFullSpeed)
   *   - loopTime: seconds between periodic() calls (e.g., 0.2s)
   *   - desiredTimeToFullSpeed: how long it should take to reach ~99% input
   *
   * Current value (0.45) is for ~1.5 seconds to full speed with 0.2s periodic loops.
   */
  private static final double DRIVE_RAMPING_GRADIENT = 0.45;

  private final CommandPS4Controller driverController = new CommandPS4Controller(0);
  private final DriveIntervalGradient driveGradientX = new DriveIntervalGradient(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND), 
        DRIVE_RAMPING_GRADIENT);
  private final DriveIntervalGradient driveGradientY = new DriveIntervalGradient(
        () -> MathUtil.applyDeadband(-driverController.getLeftX(), DEADBAND),
         DRIVE_RAMPING_GRADIENT);
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.driveCommand(
      driveGradientX,
      driveGradientY,
      () -> MathUtil.applyDeadband(driverController.getRightX(), DEADBAND)));
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

}
