package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax climberMotor;
    private SparkMaxConfig config = new SparkMaxConfig();
    public static final int climberCanID = 16;

    private SparkMaxConfig launcherConfig = new SparkMaxConfig();
    private final SparkMax launcherMotor;
    public static final int launcherCanID = 24;


    public ClimberIOReal() {
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        climberMotor = new SparkMax(climberCanID, MotorType.kBrushless);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        launcherConfig.idleMode(IdleMode.kBrake);
        launcherMotor = new SparkMax(launcherCanID, MotorType.kBrushless);


    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        //Logger.recordOutput(null, climberMotor.);
    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts/1.2);
    }

    @Override
    public void moveLauncher(double volts) {
        launcherMotor.setVoltage(volts);
    }
}