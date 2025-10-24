package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  private static final double DEADBAND = 0.1;

  private final CommandPS4Controller driverController = new CommandPS4Controller(0);
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getLeftX(), DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getRightX(), DEADBAND)));

    driverController.cross().onTrue(swerve.getResetGyro());
    driverController.L1()
      .onTrue(swerve.setFieldRelativeDrive(true))
      .onFalse(swerve.setFieldRelativeDrive(false));
    driverController.R1()
      .onTrue(swerve.setSportsMode(true))
      .onFalse(swerve.setSportsMode(false));
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

}
