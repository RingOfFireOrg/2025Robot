package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

public class AlignWithReef2D extends Command {
    double tagNum, tagAngle, angle, yaw, area, targetYaw, targetArea, maxTurnPower, maxSpeedPower, maxStrafePower;
    double VISION_TURN_ARRIVE_OFFSET = 0;
    double VISION_SPEED_ARRIVE_OFFSET = 0;
    double VISION_STRAFE_ARRIVE_OFFSET = 0;

    boolean leftReef;

    Drive drive;

    boolean endCMD = false;
    
    PIDController visionTurnController = new PIDController(
        0.05,
        0, 
        0
    );

    PIDController visionSpeedController = new PIDController(
        0.05,
        0, 
        0
    );

    PIDController visionStrafeController = new PIDController(
        0.05,
        0, 
        0
    );
    CommandXboxController driver = new CommandXboxController(0);

  public AlignWithReef2D(Drive drive, boolean leftReef) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.leftReef = leftReef;
    visionTurnController.enableContinuousInput(-180, 180);
    addRequirements(drive);

  }

  @Override
  public void initialize() {
        tagNum = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);




    if (tagNum == 18 || tagNum == 7) {
        visionTurnController.setSetpoint(0);
      tagAngle = 0;

    }
    else if (tagNum == 19 || tagNum == 6) {
        visionTurnController.setSetpoint(60);
      tagAngle = 60;

    }
    else if (tagNum == 20 || tagNum == 11) {
        visionTurnController.setSetpoint(120);
      tagAngle = 120;

    }
    else if (tagNum == 21 || tagNum == 10) {
        visionTurnController.setSetpoint(180);
      tagAngle = 180;

    }
    else if (tagNum == 22 || tagNum == 9) {
        visionTurnController.setSetpoint(240);
      tagAngle = 210;

    }
    else if (tagNum == 17 || tagNum == 8) {
        visionTurnController.setSetpoint(300);
      tagAngle = 200;

    }
    else {
        visionTurnController.setSetpoint(0);
      tagAngle = -1;
    }
  }

  @Override
  public void execute() {
    if (tagAngle == -1) {
      endCMD = true;
    }


    // double turnPower = visionTurnController.calculate(getPose().getRotation().getDegrees(), angle);
    // turnPower = MathUtil.clamp(turnPower, -maxTurnPower, maxTurnPower);

    // double deltaAngle = Math.deltaAngle(getPose().getRotation().getDegrees(), angle);
    //boolean turnOnTarget = Math.abs(deltaAngle) < VISION_TURN_ARRIVE_OFFSET;

    double speedPower = visionSpeedController.calculate(area, targetArea);
    speedPower = MathUtil.clamp(speedPower, -maxSpeedPower, maxSpeedPower);

    double deltaArea = targetArea - area;
   // boolean speedOnTarget = Math.abs(deltaArea) < VISION_SPEED_ARRIVE_OFFSET;

    double strafePower = visionStrafeController.calculate(yaw, targetYaw);
    strafePower = MathUtil.clamp(strafePower, -maxStrafePower, maxStrafePower);

    double deltaYaw = targetYaw - yaw;

    //change left and right
    //driver.x().whileTrue(DriveCommands.joystickDriveAtAngle(drive, null, null, () -> new Rotation2d(tagAngle)));

    
    //boolean strafeOnTarget = Math.abs(deltaYaw) < VISION_STRAFE_ARRIVE_OFFSET;

    //defaultDrive(-speedPower, -strafePower, -turnPower);

    // visionAngle = getPose().getRotation().getDegrees();
    // visionTargetAngle = angle;
    // visionYaw = yaw;
    // visionTargetYaw = targetYaw;
    // visionArea = area;
    // visionTargetArea = targetArea;

    //return turnOnTarget && speedOnTarget && strafeOnTarget;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return endCMD;
  }
}
