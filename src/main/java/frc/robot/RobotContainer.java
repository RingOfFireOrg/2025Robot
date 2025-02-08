package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AprilTagLineup;
import frc.robot.commands.SwerveAltJoystick;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import frc.robot.commands.TeleopCommands.SwerveNewJoystick;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);

  public RobotContainer() {
    defaultCommands();
    configureButtonBindings();
  }

  /* Sets default commands for each subsystem */
  private void defaultCommands() {

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
    //   swerveSubsystem,

    //   // Left Joystick Field Oriented
    //   () -> -driverController.getRawAxis(OIConstants.leftStickY),
    //   () -> driverController.getRawAxis(OIConstants.leftStickX),

    //   //Right Joystick For Robot Centic
    //   () -> -driverController.getRawAxis(OIConstants.rightStickY),
    //   () -> driverController.getRawAxis(OIConstants.rightStickX),

    //   // Triggers for turning
    //   () -> driverController.getRawAxis(OIConstants.rightTrigger),
    //   () -> driverController.getRawAxis(OIConstants.leftTrigger),

    //   () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),

    //   // // Speed Buttons
    //   () -> driverController.getRawButton(OIConstants.aButton),
    //   () -> driverController.getRawButton(OIConstants.bButton),
    //   () -> driverController.getRawButton(OIConstants.xButton),
    //   () -> driverController.getRawButton(OIConstants.yButton)

    // ));

    swerveSubsystem.setDefaultCommand(new SwerveAltJoystick(
      swerveSubsystem,

      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () ->driverController.getRightX()

      // () -> (-MathUtil.clamp(driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult)) 
      // - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult),0.1))
      // + (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult),0.1)),

      // () -> MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult) 
      // - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult),0.1))
      // + (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult),0.1)),
        
      // () -> MathUtil.clamp(driverController.getRightX(), -SwerveConstants.turnMult,SwerveConstants.turnMult)
      // + (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-SwerveConstants.turnMult,SwerveConstants.turnMult),0.1))
      // - (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-SwerveConstants.turnMult,SwerveConstants.turnMult),0.1))

    ));
  }


  /* Create button Bindings*/
  private void configureButtonBindings() {

    new JoystickButton(driverController, XboxController.Button.kA.value)
      .whileTrue(new AprilTagLineup(swerveSubsystem));

    // new JoystickButton(driverController, XboxController.Button.kY.value)
    //   .whileTrue(AutoBuilder.pathfindThenFollowPath(pathName(),new PathConstraints( 3.0, 4.0,
    //   Units.degreesToRadians(540), Units.degreesToRadians(720))));

    // new POVButton(driverController, 0)
    //   .whileTrue(swerveSubsystem.dPadDriveCMD(0.2, 0));
    // new POVButton(driverController, 90)
    //  .whileTrue(swerveSubsystem.dPadDriveCMD(0, -0.2) );
    // new POVButton(driverController, 180)
    //  .whileTrue(swerveSubsystem.dPadDriveCMD(-0.2, 0) );
    // new POVButton(driverController, 270)
    //  .whileTrue(swerveSubsystem.dPadDriveCMD(0, 0.2));
  
    
  } 


  

  public Command getAutonomousCommand() {
    /* Run no Auto */
    //return new InstantCommand();
    
    return new PathPlannerAuto("part1")
    // created named command to get end effector and algae intake up
    .andThen(new AprilTagLineup(swerveSubsystem))
    //.andThen(new PathPlannerAuto("part2"))
    ;
 
  }



  public PathPlannerPath pathName() {
    double tagNum = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0);
    String pathName;
    PathPlannerPath path;

    // if (tagNum == 18 || tagNum == 10) {
    //   pathName = "pf_AD Front";

    // }
    // else if (tagNum == 19 || tagNum == 9) {
    //   pathName = "pf_LK LeftFront";

    // }
    // else if (tagNum == 20 || tagNum == 8) {
    //   pathName = "pf_JI LeftBack";

    // }
    // else if (tagNum == 21 || tagNum == 7) {
    //   pathName = "pf_HG Back";

    // }
    // else if (tagNum == 22 || tagNum == 6) {
    //   pathName = "pf_EF RightBack";

    // }
    // else if (tagNum == 17 || tagNum == 11) {
    //   pathName = "pf_CD RightFront";

    // }
    // else {
    //   pathName = " ";
    // }

    pathName =  FieldConstants.pathFindHashMap.get(FieldConstants.tagToReefSide.get(tagNum));

    try {
      path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } 
    catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    return null;
  }



 

}
