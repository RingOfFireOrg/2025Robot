package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  SwerveSubsystem swerveSubsystem;

  
  @Override
  public void robotInit() {

    /* ------ Uncomment if you want the USB camera back ------ */
    // Thread m_visionThread = new Thread(
    // () -> {
    //   UsbCamera camera = CameraServer.startAutomaticCapture();
    //   camera.setResolution(640, 480);
    //   CvSink cvSink = CameraServer.getVideo();
    //   CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
    //   Mat mat = new Mat();
    //   while (!Thread.interrupted()) {
    //     if (cvSink.grabFrame(mat) == 0) {
    //       outputStream.notifyError(cvSink.getError());
    //       continue;
    //     }
    //     Imgproc.rectangle(
    //         mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
    //     outputStream.putFrame(mat);
    //   }
    // });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();
    /* ---------------------------------------------------------------------------------------- */

    m_robotContainer = new RobotContainer();    
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();    
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }  

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
    
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }

  

}
