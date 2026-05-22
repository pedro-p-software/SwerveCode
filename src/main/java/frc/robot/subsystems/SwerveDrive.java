// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

  SwerveDriveKinematics kinematics;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {


    kinematics = new SwerveDriveKinematics(

      //localizaçoes dos centros das rodas em relação ao centro do robô (encoder)

      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10))
    );

    
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
