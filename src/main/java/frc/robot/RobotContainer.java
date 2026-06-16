// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
 
  private final CommandPS4Controller controle = new CommandPS4Controller(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> controle.getLeftY() * -1,
                                                                () -> controle.getLeftX() * -1)
                                                            .withControllerRotationAxis(controle::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controle::getRightX,
                                                                                             controle::getRightY)
                                                           .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -controle.getLeftY(),
                                                                        () -> -controle.getLeftX())
                                                                    .withControllerRotationAxis(() -> controle.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);                                                             
  
SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
.withControllerHeadingAxis(() -> Math.sin(controle.getRawAxis(2) *Math.PI) *(Math.PI *2),
() -> Math.cos(controle.getRawAxis(2) *Math.PI) *(Math.PI *2)).
headingWhile(true).
translationHeadingOffset(true).
translationHeadingOffset(Rotation2d.fromDegrees(0));

/** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyro)
                                                    .andThen(Commands.none()));

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyro).withTimeout(.2)
                                                .andThen(drivebase.driveForward().withTimeout(1)));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    if (autoChooser.getSelected() == null ) {
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyro));
  }
  }
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);    
    
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      controle.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      controle.share().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      controle.options().whileTrue(drivebase.centerModulesCommand());
      controle.L2().onTrue(Commands.none());
      controle.R2().onTrue(Commands.none());
    } else
    {
      controle.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      controle.share().whileTrue(Commands.none());
      controle.options().whileTrue(Commands.none());
      controle.L2().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      controle.R2().onTrue(Commands.none());
    }

  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
