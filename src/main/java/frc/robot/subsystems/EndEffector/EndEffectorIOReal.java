package frc.robot.subsystems.EndEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.EndEffectorIntakeState;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorIOReal implements EndEffectorIO {

    private final SparkMax EndEffectorMotor;
    private final SparkMax EjectMotor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController;

    private static final int EndEffectorCanID = 17;
    private static final int EjectCanID = 18;

    private final DutyCycleEncoder absEncoder;
    private double zeroOffset = 0.0;
    private int AbsEncoderDIOID = 1;
    private double targetAngle = 0.0; // Target position in degrees

    private double MAX_VELOCITY = 10.0; // in degrees per second
    private double MAX_ACCELERATION = 360.0; // degrees per second squared
    private final ProfiledPIDController pidController = new ProfiledPIDController(
        0.05, 0.0, 0.01, // PID gains (adjust as needed)
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    private boolean enableHoming = false; // In case operator Takes manuel control


    public EndEffectorIOReal() {

        EndEffectorMotor = new SparkMax(EndEffectorCanID, MotorType.kBrushless);
        EjectMotor = new SparkMax(EjectCanID, MotorType.kBrushless);

        absEncoder = new DutyCycleEncoder(AbsEncoderDIOID);
        

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);


        config.encoder.positionConversionFactor(1);
        //config.encoder.velocityConversionFactor(1);

        EndEffectorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        EjectMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = EndEffectorMotor.getEncoder();
        closedLoopController = EndEffectorMotor.getClosedLoopController();


    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        if (enableHoming) {
            double output = pidController.calculate(getPositionDegrees(), targetAngle);
            EndEffectorMotor.set(output);
        }


        Logger.recordOutput("test_AbsEncoder", absEncoder.get());
        Logger.recordOutput("test_AbsEncoder Degrees", getPositionDegrees());

    }

    @Override
    public void setVoltage(double volts) {
        enableHoming = false;
        EndEffectorMotor.setVoltage(volts/3);
    }

    @Override
    public void setVoltageEject(double volts, double ejectsVolts) {
        enableHoming = false;
        EndEffectorMotor.setVoltage(volts/3);
        EjectMotor.setVoltage(ejectsVolts/5);
    }

    @Override
    public void setAngle(double angle) {
        targetAngle = angle;
        enableHoming = true;
    }

    @Override
    public void ejecter(double volts) {
        EjectMotor.setVoltage(volts/5);
    }

    public void endEffectorState(double volts, EndEffectorIntakeState state) {
        switch (state) {
            case EJECT:
                break;
            case INTAKE:
                break;
            case NONE:
                break;
            default:
                System.out.println("Unknown state: " + state);
        }

    }
         


    public void resetEncoder() {
        zeroOffset = absEncoder.get() * 360.0;
    }

    public double getPositionDegrees() {
        return (absEncoder.get() * 360.0) - zeroOffset;
    }


}