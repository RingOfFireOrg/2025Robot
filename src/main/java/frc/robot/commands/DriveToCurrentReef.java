package frc.robot.commands;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class DriveToCurrentReef extends Command {
  Drive drive;
  Vision vision;
  double visibleTag;
  String side;
  CommandXboxController controller = new CommandXboxController(0);
  private PIDController tagTurnController = new PIDController(3, 0, 0.5); 
  private PIDController tagForwardController = new PIDController(7, 0, 0.3); 
  double ySpeed = 0;
  double tagNum;

  public DriveToCurrentReef(Drive drive,Vision vision, String side) {
    this.vision = vision;
    this.side = side;
    this.drive = drive;
    addRequirements(vision);
    tagTurnController.enableContinuousInput(0, 360);
    tagTurnController.setTolerance(3);

    tagForwardController.setSetpoint(14);
    tagForwardController.setTolerance(2);
  }

  @Override
  public void initialize() {
    visibleTag = vision.getTargetID(0);
    tagTurnController.setSetpoint(0);
    tagForwardController.setSetpoint(14);
    tagNum = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);




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
}