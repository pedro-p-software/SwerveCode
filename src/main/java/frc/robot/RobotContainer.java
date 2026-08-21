// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.SwerveDrive;

public class RobotContainer {
 
  private final PS4Controller controle = new PS4Controller(0);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File
  (Filesystem.getDeployDirectory(),"swerve"));

/** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  
  }
  private void configureBindings() {

    //driva
    drivebase.setDefaultCommand(drivebase.newDrive(controle));
      // trava o swerve
      new JoystickButton(controle, PS4Controller.Button.kSquare.value).whileTrue(drivebase.lockSwerve());

      //reseta o gyro
      new JoystickButton(controle, PS4Controller.Button.kTriangle.value).onTrue((Commands.runOnce(drivebase::resetGyro)));

      //deixa todos modulos em 0 graus
      new JoystickButton(controle, PS4Controller.Button.kOptions.value).whileTrue(drivebase.centerModulesCommand());
  
      // Setta speeds pra direcao e turn
      new JoystickButton(controle, PS4Controller.Button.kL2.value).whileTrue(drivebase.goAndTurn());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutoCommand();
  }
}