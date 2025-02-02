package frc.robot.commands.TeleopCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final Supplier<Double> xSpdFunctionField, ySpdFunctionField, xSpdFunctionRobot, ySpdFunctionRobot, turningSpdFunctionLeft, turningSpdFunctionRight;
    private final Supplier<Boolean> aButton, bButton, xButton, yButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final XboxController driveController = new XboxController(0);
    private double speedDivide = 2;
    

    public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, 
            Supplier<Double> xSpdFunctionField, 
            Supplier<Double> ySpdFunctionField, 

            Supplier<Double> xSpdFunctionRobot,
            Supplier<Double> ySpdFunctionRobot,

            Supplier<Double> turningSpdFunctionLeft,
            Supplier<Double> turningSpdFunctionRight,
            Supplier<Boolean> fieldOrientedFunction, 

            Supplier<Boolean> aButton,
            Supplier<Boolean> bButton,
            Supplier<Boolean> xButton,
            Supplier<Boolean> yButton
            ){

        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunctionField = xSpdFunctionField;
        this.ySpdFunctionField = ySpdFunctionField;

        this.xSpdFunctionRobot = xSpdFunctionRobot;
        this.ySpdFunctionRobot = ySpdFunctionRobot;

        this.turningSpdFunctionLeft = turningSpdFunctionLeft;
        this.turningSpdFunctionRight = turningSpdFunctionRight;

        this.aButton = aButton;
        this.bButton = bButton;
        this.xButton = xButton;
        this.yButton = yButton;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        /* Reset Gyro Button (Not created as a trigger so the Swerve Drive Command does not get intrerrupted while driving) */
        if(driveController.getRawButton(7) == true) {
            swerveSubsystem.zeroHeading();
        }

        if(aButton.get() == true) {
            /* 50% Speed */
            speedDivide = 2;
        }
        if(xButton.get() == true) {
            /* 25% Speed */
            speedDivide = 4;
        }
        if(bButton.get() == true) {
            /* 75% Speed */
            speedDivide = 1.3333;
        }
        if(yButton.get() == true) {
            /* 100% Speed */
            speedDivide = 1; 
        }

        if (xSpdFunctionField.get() >= 0.1 || xSpdFunctionField.get() <= -0.1 || ySpdFunctionField.get() >= 0.1 || ySpdFunctionField.get() <= -0.1) 
        {
            //test & delete 
            if(aButton.get() == true) {
                speedDivide = 2;
            }
            if(xButton.get() == true) {
                speedDivide = 4;
            }
            if(bButton.get() == true) {
                speedDivide = 1.3333;
            }
            if(yButton.get() == true) {
                speedDivide = 1;
            }

            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunctionField.get()/speedDivide;
            double ySpeed = ySpdFunctionField.get()/speedDivide;
            double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;


            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            


            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);
        }
        else if (xSpdFunctionRobot.get() >= 0.1 || xSpdFunctionRobot.get() <= -0.1 || ySpdFunctionRobot.get() >= 0.1 || ySpdFunctionRobot.get() <= -0.1)
        {
            //test & delete
            if(aButton.get() == true) {
                speedDivide = 2;
            }
            if(xButton.get() == true) {
                speedDivide = 4;
            }
            if(bButton.get() == true) {
                speedDivide = 1.3333;
            }
            if(yButton.get() == true) {
                speedDivide = 1;
            }

            // 1. Get real-time joystick inputs
            double xSpeed = xSpdFunctionRobot.get()/speedDivide;
            double ySpeed = ySpdFunctionRobot.get()/speedDivide;
            //double ySpeed = 0;

            double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

            // 5. Convert chassis speeds to individual module states
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            swerveSubsystem.setModuleStates(moduleStates);

        }
        else {
            /* Joystick Input */
            //double xSpeed = xSpdFunctionRobot.get()/speedDivide;
            //double ySpeed = ySpdFunctionRobot.get()/speedDivide;
            double xSpeed = 0;
            double ySpeed = 0;
            double turningSpeed = turningSpdFunctionLeft.get() - turningSpdFunctionRight.get();

            /* Deadband */
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            /* Ratelimiter/Acceleration Limit */
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                    * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

            /* Create Chassis speeds for each module */
            ChassisSpeeds chassisSpeeds;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            /* Output Chassis Speeds */
            swerveSubsystem.setModuleStates(moduleStates); 
        }

    }



    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}