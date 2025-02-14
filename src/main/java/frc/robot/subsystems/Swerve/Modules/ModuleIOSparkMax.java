package frc.robot.subsystems.Swerve.Modules;


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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class ModuleIOSparkMax implements ModuleIO {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder  driveEncoder;
    private final RelativeEncoder  turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private int encId;




    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    public ModuleIOSparkMax(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;


        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

    
        driveConfig.smartCurrentLimit(40);
        driveConfig.inverted(driveMotorReversed);
        driveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turnConfig.smartCurrentLimit(40);
        turnConfig.inverted(turningMotorReversed);
        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //turningEncoder.setIntegratedSensorPosition(absoluteEncoder., timeoutMs)

        encId = absoluteEncoderId;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);


        resetEncoders();
    }

    @Override 
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.getDrivePosition = getDrivePosition();
        inputs.getTurningPosition = getTurningPosition();
        inputs.getDriveVelocity = getDriveVelocity();
        inputs.getAbsEncoderRad = getAbsoluteEncoderRad();
        inputs.getState = getState();
    } 
    
    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    }

    @Override
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    @Override
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

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }
  
}