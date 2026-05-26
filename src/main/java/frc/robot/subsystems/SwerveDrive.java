// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;

public class SwerveDrive extends SubsystemBase {

  AHRS gyro;
  SwerveModule[] swerveModule;

    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    swervelib.SwerveDrive swerveDrive;
    
    private final Pose2d startingPose = new Pose2d(new Translation2d(Meter.of(0),
                                                                     Meter.of(0)),
                                                            Rotation2d.fromDegrees(gyro.getAngle()));


  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    gyro = new AHRS(null);
    
  }


  public void drive(){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


public swervelib.SwerveDrive getSwerveDrive() {
    return swerveDrive;

    }


public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
}
public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(()->{
      swerveDrive.driveFieldOriented(velocity.get());;
    });

}

}
