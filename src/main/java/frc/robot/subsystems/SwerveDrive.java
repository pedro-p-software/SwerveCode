// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;

public class SwerveDrive extends SubsystemBase {

  AHRS gyro;

    File directory = new File(Filesystem.getDeployDirectory(),"swerve");
    
    swervelib.SwerveDrive swerveDrives;
    
    private final Pose2d startingPose = new Pose2d(new Translation2d(Meter.of(1),
                                                                     Meter.of(4)),
                                                            Rotation2d.fromDegrees(gyro.getAngle()));


  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    try
    {
      swerveDrives = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    
  }


  public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return run(() -> {
      swerveDrives.drive(new Translation2d(xSpeed.getAsDouble(), ySpeed.getAsDouble()), rot.getAsDouble(),true,false);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


public swervelib.SwerveDrive getSwerveDrive() {
    return swerveDrives;

    }




}

