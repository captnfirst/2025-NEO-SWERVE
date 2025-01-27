// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  final CommandXboxController driverXbox = new CommandXboxController(0);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -0.8,
      () -> driverXbox.getLeftX() * -0.5)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.start().whileTrue(Commands.none());
    driverXbox.back().whileTrue(Commands.none());
    driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverXbox.rightBumper().onTrue(Commands.none());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
