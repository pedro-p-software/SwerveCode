// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {


    private final SwerveDrive swerveDrive;
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final PIDController headingController = new PIDController(0.5, 0, 0);
    private double posX;
    private double posY;
    private double posAng;
    
      /** Creates a new SwerveDrive. */
      public SwerveSubsystem(File directory) {
    
        Pose2d startingPose = new Pose2d(
        new Translation2d(
        Meter.of(1),
        Meter.of(4)),
        Rotation2d.fromDegrees(0));
    
      SwerveDriveConfig cfg = new SwerveDriveConfig()
      .withSubsystem(this)
      .withStartingPose(startingPose)
      .withGyro(() -> gyro.getRotation2d().getMeasure())
      .withMaximumChassisSpeed(MetersPerSecond.of(Constants.MAX_SPEED), RadiansPerSecond.of(2 * Math.PI))
      .withRotationController(headingController)
      .withTelemetry(TelemetryVerbosity.HIGH);
      headingController.enableContinuousInput(-Math.PI, Math.PI);

      try
        {
          swerveDrive = new SwerveParser(directory).createSwerveDrive(cfg);
    
        } catch (Exception e){
    
          throw new RuntimeException(e);
    
        }
        
        SmartDashboard.setDefaultNumber("AutoMove/goToX", 0);
        SmartDashboard.setDefaultNumber("AutoMove/goToY", 0);
        SmartDashboard.setDefaultNumber("AutoMove/goToAng", 0);

        try {
          AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotVelocity, 
            (speeds) -> swerveDrive.drive(()-> speeds), 
            null, 
            RobotConfig.fromGUISettings(), 
            () -> DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false), this);

        } catch (IOException | ParseException e) {
          e.printStackTrace();
        }
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModule[] modules = swerveDrive.getConfig().getModules();
    for (int i = 0; i < modules.length; i++) {
      SmartDashboard.putNumber("Module " + (i + 1) + " Velocity",
          modules[i].getState().speedMetersPerSecond);
    }

    swerveDrive.updateTelemetry();


    posX = SmartDashboard.getNumber("AutoMove/goToX", posX);
    posY = SmartDashboard.getNumber("AutoMove/goToY", posY);
    posAng = SmartDashboard.getNumber("AutoMove/goToAng", posAng);
  }

  public Command centerModulesCommand()
  {
    return run(() -> swerveDrive.setSwerveModuleStates(
        Arrays.stream(swerveDrive.getModuleStates())
            .map(state -> new edu.wpi.first.math.kinematics.SwerveModuleState(
                0, Rotation2d.kZero))
            .toArray(edu.wpi.first.math.kinematics.SwerveModuleState[]::new)));
  }
//Comando comentado por motivos de teste
//  public Command goAndTurn(double x, double y, double angle)
  //{
    //resetGyro();
    //return run(()->{
      //headingController.enableContinuousInput(-180, 180);
      //double current = getHeading().getDegrees();
      //double output = headingController.calculate(current, angle);
      //swerveDrive.drive(new Translation2d(x,y), output, true, false);
    //}
    //).until(()-> Math.abs(headingController.getError()) < 2);
  //}

  //Confio mais nesse gosto mais
    public Command goAndTurn(){
      resetGyro();
      return run(()->{
        double current = getHeading().getRadians();
        double angOutput = headingController.calculate(current, Math.toRadians(posAng));
        swerveDrive.setFieldRelativeChassisSpeeds(
            new ChassisSpeeds(posX, posY, angOutput));
      }).until(()-> Math.abs(MathUtil.angleModulus(getHeading().getRadians() - Math.toRadians(posAng))) < Math.toRadians(2));
    }
  //A diferernça é que o 1° usa um pid manual e o 2° o pid nativo do yagsl (se usar o 1° desativa a headingcorrection)
  //funciona
  public Command turnCommand(double speed){
   
    return run(()->{
      swerveDrive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0, 0, speed));
  });}

  public Command oldDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      Translation2d translation = SwerveDriveConfig.scaleTranslation(
          new Translation2d(
              translationX.getAsDouble() * Constants.MAX_SPEED,
              translationY.getAsDouble() * Constants.MAX_SPEED),
          0.8);
      swerveDrive.setFieldRelativeChassisSpeeds(new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          Math.pow(angularRotationX.getAsDouble(), 3) * 2 * Math.PI));
    });
  }

  public Command newDrive(PS4Controller controller)
  {
    SwerveInputStream driveInput = SwerveInputStream.of(
        swerveDrive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX())
        .withAllianceRelativeControl()
        .withControllerRotationAxis(controller::getRightX)
        .withDeadband(Constants.OperatorConstants.DEADBAND)
        .withScaleTranslation(0.8)
        .withScaleRotation(0.6)
        .withCubeTranslationControllerAxis();

    SwerveInputStream headingInput = driveInput.clone()
        .withControllerHeadingAxis(controller::getRightX, controller::getRightY)
        .withHeadingControl(controller::getR3Button);
    return swerveDrive.drive(headingInput);
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.setFieldRelativeChassisSpeeds(velocity);
  }

  public SwerveDrive getSwerveDrive() {
    
    return swerveDrive;

  }

  public SwerveDriveKinematics getKinematics()
{
  return swerveDrive.getKinematics();
}

public void resetOdometry(Pose2d initialHolonomicPose)
{
  swerveDrive.resetOdometry(initialHolonomicPose);
}

public Pose2d getPose()
{
  return swerveDrive.getPose();
}

public void resetGyro()
{
  swerveDrive.zeroGyro();
}

public void setMotorBrake(boolean brake){
  throw new UnsupportedOperationException("YAGSL 2026.8.18 configures idle mode per motor in the module configuration.");
}

public Rotation2d getHeading()
{
  return getPose().getRotation();
}

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
{
  Translation2d scaledInputs = SwerveDriveConfig.cubeTranslation(new Translation2d(xInput, yInput));
  double targetHeading = Math.atan2(headingY, headingX);
  double angularVelocity = Math.hypot(headingX, headingY) < 1e-6
      ? 0
      : headingController.calculate(getHeading().getRadians(), targetHeading);
  return ChassisSpeeds.fromFieldRelativeSpeeds(
      scaledInputs.getX() * Constants.MAX_SPEED,
      scaledInputs.getY() * Constants.MAX_SPEED,
      angularVelocity,
      getHeading());
}

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
{
  Translation2d scaledInputs = SwerveDriveConfig.cubeTranslation(new Translation2d(xInput, yInput));
  return ChassisSpeeds.fromFieldRelativeSpeeds(
      scaledInputs.getX() * Constants.MAX_SPEED,
      scaledInputs.getY() * Constants.MAX_SPEED,
      headingController.calculate(getHeading().getRadians(), angle.getRadians()),
      getHeading());
}

public ChassisSpeeds getFieldVelocity()
{
  return swerveDrive.getFieldRelativeSpeed();
}

public ChassisSpeeds getRobotVelocity()
{
  return swerveDrive.getRobotRelativeSpeed();
}

public PIDController getSwerveController()
{
  return headingController;
}

public SwerveDriveConfig getSwerveDriveConfiguration()
{
  return swerveDrive.getConfig();
}

public Command lockSwerve(){
  return this.run(
    ()->swerveDrive.lockPose());
  } 

public Rotation2d getPitch()
{
  throw new UnsupportedOperationException("The custom ADXRS450 gyro supplies yaw only.");
}
}
