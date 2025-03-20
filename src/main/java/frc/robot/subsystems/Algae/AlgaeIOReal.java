package frc.robot.subsystems.Algae;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;

public class AlgaeIOReal implements AlgaeIO {

    private final SparkMax algaePivotMotor;
    private final SparkMax leftAlgaeIntakeMotor;
    private final SparkMax rightAlgaeIntakeMotor;

    private final AbsoluteEncoder absEncoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public static final int algaePivotMotorCanID = 20;
    public static final int leftAlgaeIntakeMotorCanID = 21;
    public static final int rightAlgaeIntakeMotorCanID = 22;

    public double zeroOffset = 0.0;

    private double MAX_VELOCITY = 0.5; // in degrees per second
    private double MAX_ACCELERATION = MAX_VELOCITY*2; // degrees per second squared

    private boolean atGoal = false;
    double targetAngle = 0.35;

    private final ProfiledPIDController profiledPidController = new ProfiledPIDController(
        1.4, 0.0, 0.00, // PID gains (adjust as needed)
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    ArmFeedforward feedforward = new ArmFeedforward
    (0, .6, 0, 0);
    double output = 0;


    public AlgaeIOReal() {
        algaePivotMotor = new SparkMax(algaePivotMotorCanID, MotorType.kBrushless);
        leftAlgaeIntakeMotor = new SparkMax(leftAlgaeIntakeMotorCanID, MotorType.kBrushless);
        rightAlgaeIntakeMotor = new SparkMax(rightAlgaeIntakeMotorCanID, MotorType.kBrushless);

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .absoluteEncoder.inverted(true);
            intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
            


        algaePivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        leftAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        absEncoder = algaePivotMotor.getAbsoluteEncoder();
        

        zeroOffset = resetOffset();

        profiledPidController.disableContinuousInput();

        profiledPidController.setTolerance(0.05);

    
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        output = profiledPidController.calculate(getAbsOffset(), targetAngle);
        atGoal = profiledPidController.atGoal();
        inputs.atGoal = profiledPidController.atGoal();
        Logger.recordOutput("Algae/AlgaePivot Value", algaePivotMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput("Algae/Abs Encoder Raw", absEncoder.getPosition());
        Logger.recordOutput("Algae/Abs Encoder Offset", getAbsOffset());
        Logger.recordOutput("Algae/profiled output", output);
        System.out.println(algaePivotMotor.getAppliedOutput());

        
        if (DriverStation.isDisabled()) {
            algaePivotMotor.setVoltage(0);
        }

        if (profiledPidController.atGoal()) {
            algaePivotMotor.setVoltage(0);
        }

    }

    @Override
    public void setVoltage(double volts) {
        algaePivotMotor.setVoltage(volts/1.5);

    }
    
    @Override
    public void setVoltageIntake(double volts) {
        leftAlgaeIntakeMotor.setVoltage(volts/1.2);
        rightAlgaeIntakeMotor.setVoltage(-volts/1.2);
    }

    @Override
    public void setVoltageLaunch(double left, double right) {
        leftAlgaeIntakeMotor.setVoltage(left);
        rightAlgaeIntakeMotor.setVoltage(right);
    }

    @Override
    public void setAngle(DoubleSupplier angle) {
        targetAngle = angle.getAsDouble();
        double thisOutput = profiledPidController.calculate(getAbsOffset(), angle.getAsDouble());
        System.out.println(angle);
        if (profiledPidController.atGoal()) {
            thisOutput = 0;
        }
        algaePivotMotor.setVoltage(-thisOutput*12);
        System.out.println(-thisOutput*12);
    }

    public boolean atGoal() {
        return atGoal;
    }

    public double resetOffset() {
        return absEncoder.getPosition();
    }
    public double getAbsOffset() {
        return  ((absEncoder.getPosition()- zeroOffset + (0.35)) % 1 + 1) % 1; 
    }
    public double getAbsFFOffset() {
        return  ((absEncoder.getPosition()- zeroOffset - (.2)) % 1 + 1) % 1; 
    }

}