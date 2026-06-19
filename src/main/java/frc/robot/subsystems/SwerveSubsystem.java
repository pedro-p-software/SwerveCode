// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {


    private SwerveDrive swerveDrive;
                                              
    
      /** Creates a new SwerveDrive. */
      public SwerveSubsystem(File directory) {
    
        Pose2d startingPose = new Pose2d(new Translation2d(Meter.of(1),
                                                          Meter.of(4)),
                                                          Rotation2d.fromDegrees(0));
    
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
        try
        {
          swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
    
        } catch (Exception e){
    
          throw new RuntimeException(e);
    
        }
        
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); 
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), true,
                        false);
    });
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public SwerveDrive getSwerveDrive() {
    
    return swerveDrive;

  }

  public SwerveDriveKinematics getKinematics()
{
  return swerveDrive.kinematics;
}

public void resetOdometry(Pose2d initialHolonomicPose)
{
  swerveDrive.resetOdometry(initialHolonomicPose);
}

public Pose2d getPose()
{
  return swerveDrive.getPose();
}

public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
{
  swerveDrive.setChassisSpeeds(chassisSpeeds);
}

public void postTrajectory(Trajectory trajectory)
{
  swerveDrive.postTrajectory(trajectory);
}

public void resetGyro()
{
  swerveDrive.zeroGyro();
}

public void setMotorBrake(boolean brake){

  swerveDrive.setMotorIdleMode(brake);
}

public Rotation2d getHeading()
{
  return getPose().getRotation();
}

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
{
  Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
  return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(),
      scaledInputs.getY(),
      headingX,
      headingY,
      getHeading().getRadians(),
      Constants.MAX_SPEED);
}

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
{
  Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

  return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(),
      scaledInputs.getY(),
      angle.getRadians(),
      getHeading().getRadians(),
      Constants.MAX_SPEED);
}

public ChassisSpeeds getFieldVelocity()
{
  return swerveDrive.getFieldVelocity();
}

public ChassisSpeeds getRobotVelocity()
{
  return swerveDrive.getRobotVelocity();
}

public SwerveController getSwerveController()
{
  return swerveDrive.swerveController;
}

public SwerveDriveConfiguration getSwerveDriveConfiguration()
{
  return swerveDrive.swerveDriveConfiguration;
}

public void lock()
{
  swerveDrive.lockPose();
}

public Command lockSwerve(){
  return this.run(
    ()->swerveDrive.lockPose());
  } 

public Rotation2d getPitch()
{
  return swerveDrive.getPitch();
}
}