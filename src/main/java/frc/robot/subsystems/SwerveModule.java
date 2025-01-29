package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private int encId;

    private final PIDController drivePIDController = new PIDController(
        2,
        0,
        0
    );
    private final ProfiledPIDController  turnPPIDController = new ProfiledPIDController (
        2,
        0,
        0,
        new TrapezoidProfile.Constraints(3 * Math.PI, 6 * Math.PI)

    );

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        0.1, 
        0, 
        0
    );

    private final SimpleMotorFeedforward azimuthFeedForward = new SimpleMotorFeedforward(
        0, 
        0, 
        0
    );


    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;


        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        
        //testMotor = new CANSparkMax(absoluteEncoderId, null);

        driveConfig.smartCurrentLimit(40);
        driveConfig.inverted(driveMotorReversed);
        driveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnConfig.smartCurrentLimit(40);
        turnConfig.inverted(turningMotorReversed);
        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        // driveMotor.setInverted(driveMotorReversed);
        // turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //set to coast;


        // driveMotor.setInverted(driveMotorReversed);
        // turningMotor.setInverted(turningMotorReversed);


        //turningEncoder.setIntegratedSensorPosition(absoluteEncoder., timeoutMs)

        encId = absoluteEncoderId;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        turnPPIDController.enableContinuousInput(-Math.PI, Math.PI);


        resetEncoders();
    }
    
    public double getDrivePosition() { 
       return (driveEncoder.getPosition() * ModuleConstants.kDriveMotorGearRatio) * Math.PI * ModuleConstants.kWheelDiameterMeters;
    }

    public double getTurningPosition() {
        return (getAbsoluteEncoderRad());
    }

    public double getDriveVelocity() {
        return ((driveEncoder.getPosition() / ModuleConstants.kEncoderCPR) * ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    }

    public double getAbsoluteEncoderRad() {


        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble()  * (2*Math.PI);
         
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {

        SmartDashboard.putNumber("Swerve[" + encId + "] state", Math.toDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("degrees_Swerve[" + encId + "] state", Units.radiansToDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
   

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);


                        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }

    public void runDesiredState(SwerveModuleState state) {

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("degrees_Swerve[" + encId + "] state", Units.radiansToDegrees(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
   

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

    

        // driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        double velocityRadPerSec = (state.speedMetersPerSecond * Math.cos(turnPPIDController.getPositionError())) / Units.inchesToMeters(4.0);;
        final double driveVoltage =
            drivePIDController.calculate(Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / 0.14814814814, velocityRadPerSec)
                 + driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningVoltage =
            turnPPIDController.calculate(getState().angle.getRadians(), state.angle.getRadians())
                + azimuthFeedForward.calculate(turnPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("swerve_["+encId+"] driving voltage", driveVoltage);
        SmartDashboard.putNumber("swerve_["+encId+"] turning voltage", turningVoltage);
        driveMotor.setVoltage(driveVoltage);
        turningMotor.setVoltage(turningVoltage);
    }

    public void setDesiredStatePID(SwerveModuleState state) {
        //double angle = absoluteEncoder.getAbsolutePosition();
        double driveVelocity = getDriveVelocity();
        double turningPosition = getTurningPosition();

        SmartDashboard.putNumber("Swerve[" + encId + "] state", getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Module[" + encId + "]", state.angle.getDegrees());
        SmartDashboard.putNumber("swerve_[" + encId + "] Velocity", driveMotor.getEncoder().getVelocity());
   

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);

        // driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        final double driveVoltage =
            drivePIDController.calculate(driveVelocity, state.speedMetersPerSecond)
                 + driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningVoltage =
            turnPPIDController.calculate(turningPosition, state.angle.getRadians())
                + azimuthFeedForward.calculate(turnPPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("swerve_["+encId+"] driving voltage", driveVoltage);
        SmartDashboard.putNumber("swerve_["+encId+"] turning voltage", turningVoltage);
        driveMotor.setVoltage(driveVoltage);
        turningMotor.setVoltage(turningVoltage);
    }



    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void brake(boolean doBrake){
        if(doBrake){
            driveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
            driveMotor.configure(driveConfig,SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        }
        else{
            driveConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
            driveMotor.configure(driveConfig,SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        }
        
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

    public double returnDriveMotorTemp() {
        return driveMotor.getMotorTemperature();
    }

}