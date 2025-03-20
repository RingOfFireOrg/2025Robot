package frc.robot.subsystems.Climber;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

@SuppressWarnings("unused")
public class ClimberIOReal implements ClimberIO {
    private final SparkMax climberMotor;
    private SparkMaxConfig config = new SparkMaxConfig();
    public static final int climberCanID = 16;

    public ClimberIOReal() {
        climberMotor = new SparkMax(climberCanID, MotorType.kBrushless);
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        //Logger.recordOutput(null, climberMotor.);
    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts/1.2);
    }
}