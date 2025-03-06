package frc.robot.commands;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.util.Units;
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

  public DriveToCurrentReef(Vision vision, String side) {
    this.vision = vision;
    this.side = side;
    addRequirements(vision);
  }

  @Override
  public void initialize() {
    visibleTag = vision.getTargetID(0);
  }

  @Override
  public void execute() {
    PathConstraints constraints = new PathConstraints(3.0, 4.0,Units.degreesToRadians(540), Units.degreesToRadians(720));
    if (side == "right") {
        if ((visibleTag == 18 || visibleTag == 7)) {
        try {controller.x().whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toAB"),constraints));
        } catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}}
        else if ((visibleTag == 19 || visibleTag == 6)) {
        try {controller.x().whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toCD"),constraints));
        } catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}}

    }
    else if (side == "left") {
        
    }





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