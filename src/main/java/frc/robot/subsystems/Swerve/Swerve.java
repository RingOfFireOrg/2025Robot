// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.Gyro.GyroIO;
import frc.robot.subsystems.Swerve.Modules.Module;
import frc.robot.subsystems.Swerve.Modules.ModuleIO;


public class Swerve extends SubsystemBase {
  private final Module[] swerveModules;
  private GyroIO gyroIO;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveDriveOdometry odometer;
  private SwerveModulePosition[] lastModulePositions = 
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    rawGyroRotation, 
    lastModulePositions, 
    new Pose2d()
  );
  public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    swerveModules = new Module[] {
      new Module(flModuleIO, 0),
      new Module(frModuleIO, 1),
      new Module(blModuleIO, 2),
      new Module(brModuleIO, 3),
    };
    odometer = new SwerveDriveOdometry = 
      (DriveConstants.kDriveKinematics, new Rotation2d(), getSwerveModulePosition());
    try{
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        this::driveRobotRelative,   
        new PPHolonomicDriveController(
          Constants.DriveConstants.translationConstants,
          Constants.DriveConstants.rotationConstants
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    }
    catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {

    gyroIO.updateInputs(gyroIOInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // This method will be called once per scheduler run
  }




  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
        new SwerveModulePosition(swerveModules[0].updateInputs().;, new Rotation2d(frontLeft.getTurningPosition())),
        new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
        new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
        new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition())),
    };
}

}
