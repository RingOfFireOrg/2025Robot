package frc.robot.commands.TeleopCommands;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagLineup extends Command {
  
  private final SwerveSubsystem swerveSubsystem;
  //HashMap<int a, int b> hashmap = new HashMap<>();
  private PIDController tagTurnController = new PIDController(3, 0, 0.5); 
  private PIDController tagForwardController = new PIDController(2, 0, 0.3); 
  
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  double ySpeed = 0;

  public AprilTagLineup(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    tagTurnController.enableContinuousInput(0, 360);
    tagTurnController.setTolerance(3);

    tagForwardController.enableContinuousInput(0, 100);
    tagForwardController.setTolerance(4);
    tagForwardController.setSetpoint(13);


  }

  @Override
  public void initialize() {
    tagTurnController.setSetpoint(0);
  }

  @Override
  public void execute() {


        
    double xSpeed = 0; //(100 - LimelightHelpers.getTA(Constants.VisionConstants.TagCamera))/100 ;// / 20;
    

    

    double turningSpeed = tagTurnController.calculate(swerveSubsystem.getHeading()) *Math.PI/180;
    
    double ySpeed = LimelightHelpers.getTX(Constants.VisionConstants.TagCamera)/40;
    ySpeed = MathUtil.clamp(ySpeed, -0.15, 0.15);
    if (tagTurnController.atSetpoint()) {
      xSpeed = tagForwardController.calculate(LimelightHelpers.getTA(Constants.VisionConstants.TagCamera));
      xSpeed = MathUtil.clamp(xSpeed, -0.15, 0.15);
    }



    
    
    
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
    turningSpeed = MathUtil.clamp(turningSpeed, -0.3, 0.3);

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); //robot centric
    
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // if (LimelightHelpers.getTV(Constants.VisionConstants.TagCamera))  {
    //   return true;
    // }
    // else if (LimelightHelpers.getTA(Constants.VisionConstants.TagCamera) > 50) {
    //   return true;
    // }
    // else {
    //   return false;
    // }
    return false;
  }
}
