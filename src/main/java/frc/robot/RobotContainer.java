// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final frc.robot.subsystems.SwerveDrive drivebase = new frc.robot.subsystems.SwerveDrive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller controle = new CommandPS4Controller(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
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
