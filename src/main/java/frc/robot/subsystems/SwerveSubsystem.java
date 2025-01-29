
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.LimelightHelpers;

import com.studica.frc.AHRS;
import com.pathplanner.lib.config.RobotConfig;


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
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kUSB1);
    ShuffleboardTab generateAutoTab = Shuffleboard.getTab("Generate Auto");




    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        getSwerveModulePosition());

    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(AutoConstants.kPThetaController,0.0, 0.0);





    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // RobotConfig config = RobotConfig.fromGUISettings(); 
        // try{
        //     config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     // Handle exception as needed
        //     e.printStackTrace();
        // }

        // // Configure AutoBuilder last
        // AutoBuilder.configure(
        //         this::getPose, // Robot pose supplier
        //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //                 Constants.DriveConstants.translationConstants,
        //                 Constants.DriveConstants.rotationConstants),
        //         config, // The robot configuration
        //         () -> {

        //         var alliance = DriverStation.getAlliance();
        //         if (alliance.isPresent()) {
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //         },
        //         this 
        // );
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
                Constants.DriveConstants.translationConstants
              ),
              config,
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Blue;
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
        odometer.resetPosition(new Rotation2d(Math.toRadians(gyro.getYaw())),getSwerveModulePosition(),pose); 
        
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

    public void driveRobotRelative2(ChassisSpeeds speeds) { 
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        setModuleStates(swerveModuleStates);

    }

    // ---------------------------------------------------------------------------------------------------------------

    public void fieldCentricReset() {
        gyro.reset();
    }

    public AHRS getGyro() {
        return gyro;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public PIDController getThetaController() {
        return thetaController;
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

    public void runModuleState(SwerveModuleState[] desiredStates) {
        frontLeft.runDesiredState(desiredStates[0]);
        frontRight.runDesiredState(desiredStates[1]);
        backLeft.runDesiredState(desiredStates[2]);
        backRight.runDesiredState(desiredStates[3]);
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
    
    