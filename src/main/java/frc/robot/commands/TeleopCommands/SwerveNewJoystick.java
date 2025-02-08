package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveNewJoystick extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, xSpdFunctionRobot, ySpdFunctionRobot, turningSpdFunctionLeft, turningSpdFunctionRight;
    private final Supplier<Boolean> aButton, bButton, xButton, yButton;

    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    @SuppressWarnings("unused")
    private final SlewRateLimiter xLimiterOLD, yLimiterOLD, turningLimiterOLD;
    private final XboxController driveController = new XboxController(0);
    private double speedDivide = 2;


    public SwerveNewJoystick(SwerveSubsystem swerveSubsystem, 
      Supplier<Double> xSpdFunctionField, 
      Supplier<Double> ySpdFunctionField, 

      Supplier<Double> xSpdFunctionRobot,
      Supplier<Double> ySpdFunctionRobot,

      Supplier<Double> turningSpdFunctionLeft,
      Supplier<Double> turningSpdFunctionRight,
      Supplier<Boolean> fieldOrientedFunction,
      Supplier<Boolean> aButton,
      Supplier<Boolean> bButton,
      Supplier<Boolean> xButton,
      Supplier<Boolean> yButton

      ) 
      {

        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunctionField = xSpdFunctionField;
        this.ySpdFunctionField = ySpdFunctionField;

        this.xSpdFunctionRobot = xSpdFunctionRobot;
        this.ySpdFunctionRobot = ySpdFunctionRobot;

        this.turningSpdFunctionLeft = turningSpdFunctionLeft;
        this.turningSpdFunctionRight = turningSpdFunctionRight;

        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;


        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.xLimiterOLD = new SlewRateLimiter(6.5);
        this.yLimiterOLD = new SlewRateLimiter(6.5);
        this.turningLimiterOLD = new SlewRateLimiter(5);
                
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
        
    }

    // public double signFunc(double input) {
    //   val1 = false;
    //   if
    //   return (input > 0) - (input < 0);
    // }

    @Override
    public void execute() {
      if(driveController.getRawButton(7) == true) {
        swerveSubsystem.zeroHeading();
      }

      if(aButton.get() == true) {
        /* 50% Speed */
        speedDivide = 2;
      }
      if(xButton.get() == true) {
        /* 25% Speed */
        speedDivide = 2.7;
      }
      if(bButton.get() == true) {
        /* 75% Speed */
        speedDivide = 1.3333;
      }
      if(yButton.get() == true) {
        /* 100% Speed */
        speedDivide = 1; 
      }

      if (Math.abs(xSpdFunctionField.get()) >= 0.05  || Math.abs(ySpdFunctionField.get()) >= 0.05) 
      {
        if(aButton.get() == true) {
          /* 50% Speed */
          speedDivide = 2;
        }
        if(xButton.get() == true) {
            /* 25% Speed */
            speedDivide = 4;
        }
        if(bButton.get() == true) {
            /* 75% Speed */
            speedDivide = 1.3333;
        }
        if(yButton.get() == true) {
            /* 100% Speed */
            speedDivide = 1; 
        }
        double xSpeed = xSpdFunctionField.get() / speedDivide; 
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = ySpdFunctionField.get()/ speedDivide;
        double thetaSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        //linearMagnitude = linearMagnitude * linearMagnitude;
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed,
          thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
          swerveSubsystem.getRotation2d()
        );


        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        //   linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
        //   linearVelocity.getY() * SwerveConstants.kMaxSpeed, 
        //   thetaSpeed * SwerveConstants.kMaxAngularSpeed);

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

        swerveSubsystem.setModuleStates(moduleStates);
        //swerveSubsystem.setModuleStates(moduleStates);
      }
      else if (Math.abs(xSpdFunctionRobot.get()) >= 0.05  || Math.abs(ySpdFunctionRobot.get()) >= 0.05) 
      {
        double xSpeed = xSpdFunctionRobot.get() / speedDivide; 
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = ySpdFunctionRobot.get() / speedDivide;
        double thetaSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        //linearMagnitude = linearMagnitude * linearMagnitude;
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        //   linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
        //   linearVelocity.getY() * SwerveConstants.kMaxSpeed,
        //   thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
        //   swerveSubsystem.getRotation2d()
        // );


        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed, 
          thetaSpeed * SwerveConstants.kMaxAngularSpeed);

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

        swerveSubsystem.setModuleStates(moduleStates);

      }
    else {
        /* Joystick Input */
        double xSpeed = 0;
        xSpeed = (1 / (1 - SwerveConstants.kDeadband)) * (xSpeed + ( -Math.signum(xSpeed) * SwerveConstants.kDeadband));
        double ySpeed = 0;
        double thetaSpeed = (turningSpdFunctionLeft.get() - turningSpdFunctionRight.get())*0.85;

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        thetaSpeed = turningLimiter.calculate(thetaSpeed);

        double linearMagnitude = Math.pow(MathUtil.applyDeadband(Math.hypot(xSpeed, ySpeed), SwerveConstants.kDeadband),2);
        //linearMagnitude = linearMagnitude * linearMagnitude;
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        thetaSpeed = Math.abs(thetaSpeed) > SwerveConstants.kDeadband ? thetaSpeed : 0.0;
        thetaSpeed = Math.copySign(thetaSpeed * thetaSpeed, thetaSpeed);

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
          linearVelocity.getY() * SwerveConstants.kMaxSpeed,
          thetaSpeed * SwerveConstants.kMaxAngularSpeed, 
          swerveSubsystem.getRotation2d()
        );


        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        //   linearVelocity.getX() * SwerveConstants.kMaxSpeed, 
        //   linearVelocity.getY() * SwerveConstants.kMaxSpeed, 
        //   thetaSpeed * SwerveConstants.kMaxAngularSpeed);

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);

        swerveSubsystem.setModuleStates(moduleStates);
      }

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