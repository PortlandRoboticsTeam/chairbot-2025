// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.IntervalGradient;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;


public class RobotContainer {
  Command driveCommand;
  SwerveSubsystem swerve = new SwerveSubsystem();
  Command resetGyro;
  private CommandGenericHID m_driverController = new CommandGenericHID(0);

  IntervalGradient driveGradientX = new IntervalGradient(()->MathUtil.applyDeadband( m_driverController.getRawAxis(0),Constants.DEADBAND), .9);
  IntervalGradient driveGradientY = new IntervalGradient(()->MathUtil.applyDeadband(-m_driverController.getRawAxis(1),Constants.DEADBAND), .9);
  IntervalGradient driveGradientR = new IntervalGradient(()->MathUtil.applyDeadband( m_driverController.getRawAxis(2),Constants.DEADBAND), 1);
  boolean isFieldRelativeDrive = false;
  BooleanSupplier  fieldRelativeDriveSupplier = ()->isFieldRelativeDrive;
  public RobotContainer() {
    driveCommand = swerve.driveCommand(
        driveGradientX.getValueSupplier(),
        driveGradientY.getValueSupplier(),
        driveGradientR.getValueSupplier(),
        fieldRelativeDriveSupplier
      );
      resetGyro = swerve.getResetGyro();
    // Configure the trigger bindings
    configureBindings();
    swerve.setDefaultCommand(driveCommand);
  }

  private void configureBindings() {
    m_driverController.button(1).onTrue(resetGyro);
    m_driverController.button(7).onTrue (new InstantCommand(()->{isFieldRelativeDrive= true;}));
    m_driverController.button(7).onFalse(new InstantCommand(()->{isFieldRelativeDrive=false;}));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
