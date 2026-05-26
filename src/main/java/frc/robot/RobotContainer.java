// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
 
  private final frc.robot.subsystems.SwerveDrive drivebase = new frc.robot.subsystems.SwerveDrive();

  private final CommandPS4Controller controle = new CommandPS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  private void configureBindings() {
    
    configureBindings();
  }

  //quao rapido se move
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
  ()-> controle.getLeftY() * -1, 
  ()-> controle.getLeftX() * -1 )
  .withControllerRotationAxis(controle::getRightX)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8).allianceRelativeControl(true);
  
  //pra onde se move
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                      .withControllerHeadingAxis(controle::getRightX, controle::getRightY)
                                      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
