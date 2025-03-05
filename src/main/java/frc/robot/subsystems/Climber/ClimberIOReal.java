package frc.robot.subsystems.Climber;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberIOReal implements ClimberIO {

    private final SparkMax climberMotor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController;

    MutVoltage appliedVoltage = Volts.mutable(0);
    MutAngle angle = Radians.mutable(0);
    MutAngularVelocity velocity = RadiansPerSecond.mutable(0);


    //public static final int elevatorCanIDleft = 14;
    public static final int elevatorCanID/*right*/ = 15;

    public ClimberIOReal() {


        climberMotor = new SparkMax(elevatorCanID, MotorType.kBrushless);
        config
            //.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            ;//.closedLoopRampRate(0.5);





        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        encoder = climberMotor.getEncoder();
        closedLoopController = climberMotor.getClosedLoopController();

    
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {





    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setVoltage(volts);
    }





}