package frc.robot.subsystems.Algae;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;

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

    private static final double INTAKE_POWER = 0.75; // Adjust as needed
    private static final double CURRENT_THRESHOLD = 20.0; // Adjust 
    private static final double DETECTION_TIME = 0.2; 

    private double lastDetectionTime = 0;
    private boolean ballDetected = false;
    public double zeroOffset = 0.0;

    public AlgaeIOReal() {
        algaePivotMotor = new SparkMax(algaePivotMotorCanID, MotorType.kBrushless);
        leftAlgaeIntakeMotor = new SparkMax(leftAlgaeIntakeMotorCanID, MotorType.kBrushless);
        rightAlgaeIntakeMotor = new SparkMax(rightAlgaeIntakeMotorCanID, MotorType.kBrushless);

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
            

        algaePivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        leftAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        rightAlgaeIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        absEncoder = algaePivotMotor.getAbsoluteEncoder();
        zeroOffset = resetOffset();

    
    }

    @Override
    public void updateInputs(AlgaeIOInputs inputs) {
        Logger.recordOutput("test_isTheIntakeBallin", isTheIntakeBallin());
        Logger.recordOutput("Algae/AlgaePivot Value", algaePivotMotor.getEncoder().getPosition());
        Logger.recordOutput("Algae/Abs Encoder Raw", absEncoder.getPosition());
        Logger.recordOutput("Algae/Abs Encoder Offset", absEncoder.getPosition());

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

    public boolean isTheIntakeBallin() {
        double avgCurrent = (leftAlgaeIntakeMotor.getOutputCurrent() + rightAlgaeIntakeMotor.getOutputCurrent()) / 2.0;

        if (avgCurrent > CURRENT_THRESHOLD) {
            if (!ballDetected) {
                if (Timer.getFPGATimestamp() - lastDetectionTime > DETECTION_TIME) {
                    ballDetected = true;
                }
            }
        } else {
            lastDetectionTime = Timer.getFPGATimestamp();
            ballDetected = false;
        }

        return ballDetected;
    }


    public double resetOffset() {
        return absEncoder.getPosition();
    }

}