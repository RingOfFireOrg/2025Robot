package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;


public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive drive;
  private double tagID = -1;
  ProfiledPIDController angleController = new ProfiledPIDController(DriveCommands.ANGLE_KP, 0.0, DriveCommands.ANGLE_KD, new TrapezoidProfile.Constraints(DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));


  public AlignToReefTagRelative(boolean isRightScore, Drive drive) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-tag");
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-tag") && LimelightHelpers.getFiducialID("limelight-tag") == tagID) {
      this.dontSeeTagTimer.reset();
      System.out.println(tagID);

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-tag");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);



      xSpeed = MathUtil.clamp(xSpeed, -0.3, 0.3);
      ySpeed = MathUtil.clamp(xSpeed, -0.3, 0.3);
      //rotValue = MathUtil.clamp(xSpeed, -0.3, 0.3);
      rotValue = 0;

      // ---------------------------------------------------------------------------------
      // double x = xSupplier.getAsDouble();
      // double y = ySupplier.getAsDouble();
      // double omega = omegaSupplier.getAsDouble();

      Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(xSpeed,ySpeed);

      // Square rotation value for more precise control
      rotValue = Math.copySign(rotValue * rotValue, rotValue);

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds = new ChassisSpeeds(
          linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
          linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
          rotValue * drive.getMaxAngularSpeedRadPerSec());


      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          drive.getRotation());
      drive.runVelocity(speeds);

      //drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

    if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
      stopTimer.reset();
    }
    } 
    else {
      drive.runVelocity(new ChassisSpeeds(0,0,0));
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    //drivebase.drive(new Translation2d(), 0, false);
    drive.runVelocity(new ChassisSpeeds(0,0,0));
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}