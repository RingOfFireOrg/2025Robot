
package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
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
    //private BuiltInAccelerometer builtInAccelerometer = new BuiltInAccelerometer();
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getSwerveModulePosition());
    private Field2d field = new Field2d();

    private final SwerveDrivePoseEstimator m_poseEstimator =
        new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
          VecBuilder.fill(0.5, 0.5, Math.toRadians(30))
        );

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
      
        
        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });
    }

    public void zeroHeading() {
        gyro.reset();
    }
    
    public Command resetGyro() {
        return this.runOnce(() ->  this.zeroHeading());
    }

    public AHRS getGyro() {
        return gyro;
    }

    public double getHeading() {
        //return Rotation2d.fromDegrees(-gyro.getYaw());
        double temp = (gyro.getAngle() % 360);
        if(temp < 0) {temp = temp + 360;}
        return temp;
    }

    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(getHeading());
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometer.resetPosition(new Rotation2d(Math.toRadians(gyroFlip * gyro.getYaw())),getSwerveModulePosition(),pose);    
        m_poseEstimator.resetPosition(new Rotation2d(Math.toRadians(gyroFlip * gyro.getYaw())),getSwerveModulePosition(),pose);
    }

    //2/7/25
    // public void resetOdometry(Pose2d pose) {
    //     odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    // }



    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(moduleStates);
    }

    





    @Override
    public void periodic() {

        SmartDashboard.putNumber("LEANLEANLEAN", NetworkTableInstance.getDefault().getTable(VisionConstants.TagCamera).getEntry("tid").getDouble(0));

        odometer.update(getRotation2d(), getSwerveModulePosition());
        updateOdometry();
        SmartDashboard.putNumber("tagcamera_X", (100 - LimelightHelpers.getTA(Constants.VisionConstants.TagCamera))/150);
        SmartDashboard.putNumber("swerve_Robot Heading", getHeading());
        SmartDashboard.putNumber("swerve_Robot Theta", getPose().getRotation().getDegrees());



        //SmartDashboard.putData("Swerve Pose", m_poseEstimator.getEstimatedPosition());
        field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", field);


        
    }

    public void updateOdometry() {
        m_poseEstimator.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            }
        );

        boolean useMegaTag2 = true; 
        boolean doRejectUpdate = false;
        if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation(VisionConstants.TagCamera, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.TagCamera);
            if(Math.abs(gyro.getRate()) > 720) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if(!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            }
            // if ((builtInAccelerometer.getX() == 0 && frontLeft.getDriveVelocity() > 3)) {
                /* TODO: Do this later */
            // } 
        }
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



    /* -------------------------------------- */
    public boolean checkAcceleration() {
        if (frontLeft.getDriveVelocity() > 50 ) {
            return true;
        }
        return false;
    }
    /* -------------------------------------- */


  


    
}
    
    