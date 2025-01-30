package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopCommands.AprilTagLineup;
import frc.robot.commands.TeleopCommands.SwerveAltJoystick;
import frc.robot.commands.TeleopCommands.SwerveJoystickCommand;
import frc.robot.commands.TeleopCommands.SwerveNewJoystick;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  private XboxController driverController = new XboxController(OIConstants.driverControllerPort);
  private CommandXboxController operatorController = new CommandXboxController(OIConstants.operatorControllerPort);
  //private XboxController climberController = new XboxController(OIConstants.climberControllerPort); 


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
      // Left Joystick Field Oriented
      () -> (-MathUtil.clamp(driverController.getLeftY(),-0.7,0.7)) 
        - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftY(),-0.7,0.7),0.1))
        + (driverController.getLeftTriggerAxis() *  MathUtil.clamp(-driverController.getLeftY(),-0.7,0.7)),

      () -> MathUtil.clamp(driverController.getLeftX(),-0.7,0.7) 
        - (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getLeftX(),-0.7,0.7),0.1))
        + (driverController.getLeftTriggerAxis() *  MathUtil.clamp(driverController.getLeftX(),-0.7,0.7)),

        


      () -> MathUtil.clamp(driverController.getRightX(), -0.5,0.5)
      + (MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.1) 
      *  MathUtil.applyDeadband(MathUtil.clamp(-driverController.getRightX(),-0.5,0.5),0.1))
      - (driverController.getLeftTriggerAxis() *  MathUtil.clamp(-driverController.getRightX(),-0.5,0.5))

    ));
  }

 












  /* Create button Bindings*/
  private void configureButtonBindings() {

    new JoystickButton(driverController, Constants.OIConstants.aButton)
    .whileTrue(new AprilTagLineup(swerveSubsystem));
  
  } 


  

  public Command getAutonomousCommand() {

    /* Run no Auto */
    //return new InstantCommand();
    return new PathPlannerAuto("part1")
    //.andThen(new AprilTagLineup(swerveSubsystem))
//    .andThen(new PathPlannerAuto("part2"))
    ;
 
  }






 

}
