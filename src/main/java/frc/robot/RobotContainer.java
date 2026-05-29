// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
 
  private final SwerveDrive swerveDrive = new SwerveDrive();

  private final PS4Controller controle = new PS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveDrive.drive(
    () -> -controle.getLeftY(), 
    () -> -controle.getLeftX(), 
    () -> -controle.getRightX()));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
