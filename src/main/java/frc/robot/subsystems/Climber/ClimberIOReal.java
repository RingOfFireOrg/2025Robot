package frc.robot.subsystems.Climber;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimberIOReal implements ClimberIO {

    private final SparkMax climberMotor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();



    public static final int climberCanID = 16;

    public ClimberIOReal() {


        climberMotor = new SparkMax(climberCanID, MotorType.kBrushless);
        config
            //.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            ;//.closedLoopRampRate(0.5);





        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = climberMotor.getEncoder();

    
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {





    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts/3);
    }





}