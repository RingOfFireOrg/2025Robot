package frc.robot.commands.TeleopCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagLineup extends Command {
  
  private final SwerveSubsystem swerveSubsystem;
  //HashMap<int a, int b> hashmap = new HashMap<>();
  private PIDController tagTurnController = new PIDController(3, 0, 0.5); 
  private PIDController tagForwardController = new PIDController(7, 0, 0.3); 
  
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  double ySpeed = 0;
  double tagNum;

  public AprilTagLineup(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    tagTurnController.enableContinuousInput(0, 360);
    tagTurnController.setTolerance(3);

    tagForwardController.setSetpoint(14);
    tagForwardController.setTolerance(2);


  }

  @Override
  public void initialize() {
    tagTurnController.setSetpoint(0);
    tagForwardController.setSetpoint(14);
    tagNum = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);


    // if (tagNum==17) {
    //   tagTurnController.setSetpoint(0);

    // }
    // else if (tagNum == 21) {
    //   tagTurnController.setSetpoint(300);

    // }
    // else {
    //   tagTurnController.setSetpoint(0);
    // }

    if (tagNum == 18 || tagNum == 7) {
      tagTurnController.setSetpoint(0);

    }
    else if (tagNum == 19 || tagNum == 6) {
      tagTurnController.setSetpoint(60);

    }
    else if (tagNum == 20 || tagNum == 11) {
      tagTurnController.setSetpoint(120);

    }
    else if (tagNum == 21 || tagNum == 10) {
      tagTurnController.setSetpoint(180);

    }
    else if (tagNum == 22 || tagNum == 9) {
      tagTurnController.setSetpoint(240);

    }
    else if (tagNum == 17 || tagNum == 8) {
      tagTurnController.setSetpoint(300);

    }
    else {
      tagTurnController.setSetpoint(0);
    }


  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("SetpointAngle", tagTurnController.getSetpoint());
    SmartDashboard.putBoolean("Setpoint", tagForwardController.atSetpoint());
    SmartDashboard.putNumber("tagforwardNum", LimelightHelpers.getTA(Constants.VisionConstants.TagCamera));
    SmartDashboard.putNumber("SetpointNum", tagForwardController.getSetpoint());


    double xSpeed = 0;
    double turningSpeed = tagTurnController.calculate(swerveSubsystem.getHeading()) *Math.PI/180;
    double ySpeed = LimelightHelpers.getTX(Constants.VisionConstants.TagCamera)/40;
    ySpeed = MathUtil.clamp(ySpeed, -0.15, 0.15);
    
    if (tagTurnController.atSetpoint()) {
      if (LimelightHelpers.getTA(Constants.VisionConstants.TagCamera) != 0) {
        xSpeed = tagForwardController.calculate(LimelightHelpers.getTA(Constants.VisionConstants.TagCamera))/15;
        xSpeed = MathUtil.clamp(xSpeed, -0.2, 0.2); 
    }
     if ( tagForwardController.atSetpoint()) {
      xSpeed = 0;
     }
    }
    SmartDashboard.putNumber("xSpeed Camera", xSpeed);

    turningSpeed = -MathUtil.clamp(turningSpeed, -0.3, 0.3);

    ySpeed = -ySpeed;
    
    xSpeed = Math.abs(xSpeed) > 0.05 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    
    //xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
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
    // if (tagTurnController.atSetpoint()) {
    //   if (tagForwardController.atSetpoint()) {
    //     return true;
    //   }
    // }
    if (tagTurnController.atSetpoint()) {
     if ( tagForwardController.atSetpoint()) {
      return true;
     }
    }
    return false;
  }
}
