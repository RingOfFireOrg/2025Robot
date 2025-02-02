package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAltJoystick extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final XboxController driveController = new XboxController(0);
  Translation2d linearVelocity = null; // if it doesnt work check this
  double thetaSpeed = 0;

  private PIDController leftStationRotController = new PIDController(4, 0, 1); //TODO: TUNEEEEEEEEE
  private PIDController rightStationRotController = new PIDController(4, 0, 1); //TODO: TUNEEEEEEEEE
 
  
  
  public SwerveAltJoystick(SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpdFunctionField, 
    Supplier<Double> ySpdFunctionField, 
    Supplier<Double> turningSpdFunction

  ) 
  {
    this.swerveSubsystem = swerveSubsystem;

    this.xSpdFunctionField = xSpdFunctionField;
    this.ySpdFunctionField = ySpdFunctionField;


    this.turningSpdFunction = turningSpdFunction;


    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    leftStationRotController.setSetpoint(225);
    rightStationRotController.setSetpoint(315);

    leftStationRotController.enableContinuousInput(0, 360);
    leftStationRotController.setTolerance(2);
    
    rightStationRotController.enableContinuousInput(0, 360);
    rightStationRotController.setTolerance(2);


    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
// 
  }



  @Override
  public void execute() {

    if(driveController.getRawButton(7) == true) {
      swerveSubsystem.zeroHeading();
    }

    // turning
    if (driveController.getLeftBumperButton() == true) { //turn to left source
      
      thetaSpeed = -leftStationRotController.calculate(swerveSubsystem.getHeading()) *Math.PI/180;    
      // thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.1);
      // thetaSpeed = turningLimiter.calculate(thetaSpeed);
      // thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
      // thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      // thetaSpeed = thetaSpeed * SwerveConstants.kMaxAngularSpeed;
      //thetaSpeed = MathUtil.clamp(thetaSpeed, 0.4, 0.4);


    }
    else if (driveController.getRightBumperButton() == true) { // turn to right source
      thetaSpeed =- rightStationRotController.calculate(swerveSubsystem.getHeading()) * Math.PI/180;    
      // thetaSpeed = MathUtil.applyDeadband(thetaSpeed, 0.1);
      // thetaSpeed = turningLimiter.calculate(thetaSpeed);
      // thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
      // thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      // thetaSpeed = thetaSpeed * SwerveConstants.kMaxAngularSpeed;
      //thetaSpeed = MathUtil.clamp(thetaSpeed, 0.4, 0.4);

    }
    else { // turning generic
      thetaSpeed = -turningSpdFunction.get();
      thetaSpeed = turningLimiter.calculate(thetaSpeed);
      // thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
      thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);
      thetaSpeed = thetaSpeed * SwerveConstants.kMaxAngularSpeed;
    }

   // double thetaSpeed = turningSpdFunction.get();

    double xSpeed = -xSpdFunctionField.get() / 2; 
    xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
    double ySpeed = ySpdFunctionField.get() / 2;
    ySpeed = (1 / (1 - SwerveConstants.kDeadband)) * (ySpeed + ( -Math.signum(ySpeed) * SwerveConstants.kDeadband));

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);

    SmartDashboard.putNumber("r_xspeed", xSpeed);
    SmartDashboard.putNumber("r_yspeed", ySpeed);
    SmartDashboard.putNumber("r_zspeed", thetaSpeed);

    double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
    Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
    
    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
    .getTranslation();
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
    linearVelocity.getY() * SwerveConstants.kMaxSpeed,
    thetaSpeed, 
    swerveSubsystem.getRotation2d());
    

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

    swerveSubsystem.setModuleStates(moduleStates);
  }



  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}