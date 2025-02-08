package frc.robot;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class IDConstants {
        //Swerve IDs
        public static final int frontLeftDriveMotor = 1;
        public static final int frontLeftTurnMotor = 2;
        public static final int frontLeftCANcoder = 9;

        public static final int frontRightDriveMotor = 3;
        public static final int frontRightTurnMotor = 4;
        public static final int frontRightCANcoder = 10;

        public static final int backLeftDriveMotor = 5;
        public static final int backLeftTurnMotor = 6;
        public static final int backLeftCANcoder = 11;

        public static final int backRightDriveMotor = 7;
        public static final int backRightTurnMotor = 8;
        public static final int backRightCANcoder = 12;

    }

    public static final class ModuleConstants {

        public static final double kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75; //L2
        public static final double kTurningMotorGearRatio = 1 / (150/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        /* Distance between right and left wheels */
        public static final double kTrackWidth =  Units.inchesToMeters(22.75); 
        /* Distance between front and back wheels */
        public static final double kWheelBase = Units.inchesToMeters(22.75); 

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),    // Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   // Front Right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // Back Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  //Back Right
        );

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveOffsetDegrees= (-4.5 ); //offset in degrees - CanCoder 9
        public static final double kFrontRightDriveOffsetDegrees= (-180+74 + 90 +180 +180+7); //offset in degrees - CanCoder 10
        public static final double kBackLeftDriveOffsetDegrees= (30+118+180 +180); //offset in degrees - CanCoder 11
        public static final double kBackRightDriveOffsetDegrees= (-26+180); //offset in degrees - CanCoder 12

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  Math.toRadians(kFrontLeftDriveOffsetDegrees);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kFrontRightDriveOffsetDegrees);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackLeftDriveOffsetDegrees);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(kBackRightDriveOffsetDegrees);
       

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1);
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond ; //change denomenator
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4; //change denomenator
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;

        public static final PIDConstants translationConstants = new PIDConstants(1.0, 0.0, 0.5);
        public static final PIDConstants rotationConstants = new PIDConstants(1, 0.0, 0.1);
    }

    public static final class SwerveConstants {

        /* Distance between right and left wheels */
        public static final double kTrackWidth =  Units.inchesToMeters(22.75); 
        /* Distance between front and back wheels */
        public static final double kWheelBase = Units.inchesToMeters(22.75); 

        public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
       
        public static final double kMaxSpeed = Units.feetToMeters(15.1);
        public static final double kMaxAngularSpeed = kMaxSpeed / kDriveBaseRadius;
       
        public static final double kSlewRateTranslation = 7;    
        public static final double kSlewRateRotation = 4;
        public static final double kDeadband = 0.01;

        public static final double kDriveMotorGearRatio = 1 / 6.75; //L2
        public static final double kTurningMotorGearRatio = 1 / (150/7);

        public static final double forwardMult = 0.7;
        public static final double strafeMult = 0.7;
        public static final double turnMult = 0.7;

    }


    public static final class AutoConstants {
        public static final double kMaxModuleSpeed = 4.5;

        public static final double kTranslationP = 9;
        public static final double kTranslationI = 0;
        public static final double kTranslationD = 0.5;

        public static final double kRotationP = 6;
        public static final double kRotationI = 0;
        public static final double kRotationD = 0.5;

        public static final double kDriveBaseRadius = Units.inchesToMeters(15.909905);

        public static final double kPXController = 0.75; 
        public static final double kPYController = 0.75; 
        public static final double kPThetaController = 1.75; 


    }

    public static final class OIConstants {

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final int climberControllerPort = 2; 

        public static final double kDeadband = 0.05;
        public static final double controllerDeadband = 0.2;

    }

    public static final class LEDConstants {
        
        public static int[] PyroTechRed = {105, 3, 12};
        public static int[] PyroTechOrange = {250, 64, 2}; 

        public static int[] CryoTechBlue = {44, 2, 186}; 
        public static int[] CryoTechPurple = {115, 76, 245}; 

    }

    public static final class VisionConstants {
        public static final String TagCamera = "limelight-tag";

    };



}