
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;

import com.studica.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule
    (
        IDConstants.frontLeftDriveMotor,
        IDConstants.frontLeftTurnMotor,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        IDConstants.frontLeftCANcoder,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule
    (
        IDConstants.frontRightDriveMotor,
        IDConstants.frontRightTurnMotor,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        IDConstants.frontRightCANcoder,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule
    (
        IDConstants.backLeftDriveMotor,
        IDConstants.backLeftTurnMotor,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        IDConstants.backLeftCANcoder,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule
    (
        IDConstants.backRightDriveMotor,
        IDConstants.backRightTurnMotor,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        IDConstants.backRightCANcoder,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );
    
    //private final AHRS gyro = new AHRS(SerialPort.Port.kOnboard);
    private AHRS gyro = new AHRS(AHRS.NavXComType.kUSB1);
    private double gyroFlip = -1;
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getSwerveModulePosition());

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Configure AutoBuilder
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
          }catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
          }
      
        //   // Set up custom logging to add the current path to a field 2d widget
        //   PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
      
        //   SmartDashboard.putData("Field", field);
        }
    

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        //return Rotation2d.fromDegrees(-gyro.getYaw());

        double temp = (gyro.getAngle() % 360);
        
        if(temp < 0) {
            temp = temp + 360; 
        }
        return temp;
    }



    // ---------------------------------------------------------------------------------------------------------------
    public Pose2d getPose() {
        SmartDashboard.putString( "swerve_Get Pose meters ", odometer.getPoseMeters().toString()); 
        return odometer.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        //odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
        odometer.resetPosition(new Rotation2d(Math.toRadians(gyroFlip * gyro.getYaw())),getSwerveModulePosition(),pose); 
        
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
       // return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(moduleStates);
    }

    // ---------------------------------------------------------------------------------------------------------------



    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(getHeading());
        return Rotation2d.fromDegrees(-gyro.getYaw());

    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("LEANLEANLEAN", NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid").getDouble(0));

        odometer.update(getRotation2d(), getSwerveModulePosition());
        
        SmartDashboard.putNumber("tagcamera_X", (100 - LimelightHelpers.getTA(Constants.VisionConstants.TagCamera))/150);
        SmartDashboard.putNumber("swerve_Robot Heading", getHeading());
        SmartDashboard.putNumber("swerve_Robot Theta", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerve_FL Robot Tranlsation", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("swerve_FR Robot Tranlsation", frontRight.getDrivePosition());
        SmartDashboard.putNumber("swerve_BL Robot Tranlsation", backLeft.getDrivePosition());
        SmartDashboard.putNumber("swerve_BR Robot Tranlsation", backRight.getDrivePosition());

        SmartDashboard.putNumber("swerve_FL Motor Temp", frontLeft.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_FR Motor Temp", frontRight.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BL Motor Temp", backLeft.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BR Motor Temp", backRight.returnDriveMotorTemp());

        SmartDashboard.putNumber("swerve_FL Heading", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("swerve_FR Heading", frontRight.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BL Heading", backLeft.returnDriveMotorTemp());
        SmartDashboard.putNumber("swerve_BR Heading", backRight.returnDriveMotorTemp());
        

        
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition())),
        };
    }


    public SwerveModuleState[] getModuleStates() { 
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()

        };
    }    

    

    public void driveForward(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void brake(boolean doBrake){
        if(doBrake){
            frontLeft.brake(true);
            frontRight.brake(true);
            backLeft.brake(true);
            backRight.brake(true);
        }
        else{
            frontLeft.brake(false);
            frontRight.brake(false);
            backLeft.brake(false);
            backRight.brake(false);
        }
    }
    
    public void dPadDrive(double xSpeed, double ySpeed) {
        double turningSpeed = 0;
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(moduleStates); 
    }
    public Command dPadDriveCMD(double xSpeed, double ySpeed) {
        return this.run(() -> dPadDrive(xSpeed, ySpeed));
    }

    public void drive(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public Command resetGyro() {
        return this.runOnce(() ->  this.zeroHeading());
    }



  
    /* ------------------------------------- */


    
}
    
    