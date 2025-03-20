package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class DriveToCurrentReef extends Command {
  Drive drive;
  Vision vision;
  double visibleTag;
  CommandXboxController controller = new CommandXboxController(0);
  private PIDController tagTurnController = new PIDController(3, 0, 0.5); 
  private PIDController tagForwardController = new PIDController(7, 0, 0.3); 
  double ySpeed = 0;
  double tagNum;
  double tagAngle = -1;
  double rot = 0;
  double xSpeed = 0;
  Rotation2d poseAngle;

  public DriveToCurrentReef(Drive drive,Vision vision) {
    //this.vision = vision;
    this.drive = drive;
    addRequirements(drive);
    //addRequirements(vision);

    // tagTurnController.enableContinuousInput(0, 360);
    // tagTurnController.setTolerance(3);

    // tagForwardController.setSetpoint(14);
    // tagForwardController.setTolerance(2);
  }

  @Override
  public void initialize() {
    visibleTag = vision.getTargetID(0);
    tagTurnController.setSetpoint(0);
    tagForwardController.setSetpoint(14);
    tagNum = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);




    if (tagNum == 18 || tagNum == 7) {
      tagTurnController.setSetpoint(0);
      tagAngle = 0;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));

    }
    else if (tagNum == 19 || tagNum == 6) {
      tagTurnController.setSetpoint(60);
      tagAngle = 60;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));


    }
    else if (tagNum == 20 || tagNum == 11) {
      tagTurnController.setSetpoint(120);
      tagAngle = 120;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));


    }
    else if (tagNum == 21 || tagNum == 10) {
      tagTurnController.setSetpoint(180);
      tagAngle = 180;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));


    }
    else if (tagNum == 22 || tagNum == 9) {
      tagTurnController.setSetpoint(240);
      tagAngle = 210;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));


    }
    else if (tagNum == 17 || tagNum == 8) {
      tagTurnController.setSetpoint(300);
      tagAngle = 300;
      poseAngle = new Rotation2d(Math.toRadians(tagAngle));


    }
    else {
      tagTurnController.setSetpoint(0);
      tagAngle = -1;
      poseAngle = new Rotation2d(Math.toRadians(0));

    }
  }

  @Override
  public void execute() {
    final var rot_limelight = limelight_aim_proportional();
    rot = rot_limelight;

    final var forward_limelight = limelight_range_proportional();
    xSpeed = forward_limelight;

    if (tagAngle != -1) {
       // DriveCommands.joystickDriveAtAngle(drive, () -> forward_limelight, null, () -> new Rotation2d(Math.toRadians(tagAngle)));
      //DriveCommands.joystickDrive(drive, () -> forward_limelight, () -> 0, () -> rot);
      if (Math.abs(drive.getRotation().getDegrees() - tagAngle) > 10 ) {
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, () -> poseAngle);
      }
      else {
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, () -> poseAngle);

      }
      

    }


    // PathConstraints constraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
    // if (side == "right") {
    //     if ((visibleTag == 18 || visibleTag == 7)) {
    //     try {controller.x().whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toAB"),constraints));
    //     } catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}}
    //     else if ((visibleTag == 19 || visibleTag == 6)) {
    //     try {controller.x().whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toCD"),constraints));
    //     } catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}}

    // }
    // else if (side == "left") {
        
    // }





    Logger.recordOutput("Visible Tag", visibleTag);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // if (!controllerBoolean.get()) {
    //     return true;
    // }
    return false;
  }


  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX(VisionConstants.LimelightFrontName) * kP;
    // convert to radians per second for our drive method
    targetingAngularVelocity *= drive.getMaxAngularSpeedRadPerSec();
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY(VisionConstants.LimelightFrontName) * kP;
    targetingForwardSpeed *= drive.getMaxLinearSpeedMetersPerSec();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
}