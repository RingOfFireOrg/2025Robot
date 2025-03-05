package frc.robot.subsystems.EndEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.EndEffectorIntakeState;

import com.revrobotics.spark.config.SparkMaxConfig;

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

    public static final int EndEffectorCanID = 17;
    public static final int EjectCanID = 18;


    public EndEffectorIOReal() {

        EndEffectorMotor = new SparkMax(EndEffectorCanID, MotorType.kBrushless);
        EjectMotor = new SparkMax(EjectCanID, MotorType.kBrushless);

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

    }

    @Override
    public void setVoltage(double volts) {
        EndEffectorMotor.setVoltage(volts/3);
    }

    @Override
    public void setVoltageEject(double volts, double ejectsVolts) {
        EndEffectorMotor.setVoltage(volts/2);
        EjectMotor.setVoltage(ejectsVolts/5);
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
         
    


}