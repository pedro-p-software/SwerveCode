// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  PigeonIMU gyro;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {


    kinematics = new SwerveDriveKinematics(

      //localizaçoes dos centros das rodas em relação ao centro do robô (encoder)

      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10))
    );

    gyro = new PigeonIMU(null);

    odometry = new SwerveDriveOdometry(
      kinematics,
      null,
      new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()});
      new Pose2d(0,0, new Rotation2d());
      
    
  }


  public void drive(){
    ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
    SwerveModuleState[] statesOfModules = kinematics.toSwerveModuleStates(testSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
