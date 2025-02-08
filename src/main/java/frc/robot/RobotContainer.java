package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopCommands.AprilTagLineup;
import frc.robot.commands.TeleopCommands.SwerveAltJoystick;
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

      () -> (-MathUtil.clamp(driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult)) 
      - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult),0.1))
      + (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-SwerveConstants.forwardMult,SwerveConstants.forwardMult),0.1)),

      () -> MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult) 
      - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult),0.1))
      + (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftX(),-SwerveConstants.strafeMult,SwerveConstants.strafeMult),0.1)),
        
      () -> MathUtil.clamp(driverController.getRightX(), -SwerveConstants.turnMult,SwerveConstants.turnMult)
      + (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-SwerveConstants.turnMult,SwerveConstants.turnMult),0.1))
      - (MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.1)  *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-SwerveConstants.turnMult,SwerveConstants.turnMult),0.1))

    ));
  }


  /* Create button Bindings*/
  private void configureButtonBindings() {

    new JoystickButton(driverController, XboxController.Button.kA.value)
    .whileTrue(new AprilTagLineup(swerveSubsystem));

    new POVButton(driverController, 0)
    .whileTrue(swerveSubsystem.dPadDriveCMD(0.2, 0));
    new POVButton(driverController, 90)
    .whileTrue(swerveSubsystem.dPadDriveCMD(0, -0.2) );
    new POVButton(driverController, 180)
    .whileTrue(swerveSubsystem.dPadDriveCMD(-0.2, 0) );
    new POVButton(driverController, 270)
    .whileTrue(swerveSubsystem.dPadDriveCMD(0, 0.2));
  
    
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






 

}
