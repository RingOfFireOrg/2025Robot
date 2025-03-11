package frc.robot.subsystems.Elevator;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIORealTalon implements ElevatorIO {

    private final TalonFX elevatorMotor;
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    final VoltageOut elevatorRequest;
    // MutVoltage appliedVoltage = Volts.mutable(0);
    // MutAngle angle = Radians.mutable(0);
    // MutAngularVelocity velocity = RadiansPerSecond.mutable(0);
    TalonFXConfiguration talonFXConfigs;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);


    // 1.5
    // 36:1 


    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, kG;

    //public static final int elevatorCanIDleft = 14;
    public static final int elevatorCanID/*right*/ = 15;

    public ElevatorIORealTalon() {
        elevatorMotor = new TalonFX(elevatorCanID);
        talonFXConfigs = new TalonFXConfiguration();
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

        FeedbackConfigs fdb = talonFXConfigs.Feedback;
        fdb.SensorToMechanismRatio = 36; // 36 rotor rotations per mechanism rotation
        
        MotionMagicConfigs mm = talonFXConfigs.MotionMagic;
        mm
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(10.5)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        // // set Motion Magic settings
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 4; // An error of 1 rotation results in 1 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kG = 0.1;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        // StatusCode status = StatusCode.StatusCodeNotInitialized;
        // for (int i = 0; i < 5; ++i) {
        //     status = elevatorMotor.getConfigurator().apply(talonFXConfigs);
        //     if (status.isOK()) break;
        // }
        // if (!status.isOK()) {
        //     System.out.println("Could not configure device. Error: " + status.toString());
        // }

        elevatorRequest = new VoltageOut(0.0);
        var slot1 = talonFXConfigs.Slot1;
        slot1.GravityType = GravityTypeValue.Elevator_Static;
        kG = 1;
        slot1.kG = kG; // Replace with your determined value
        elevatorMotor.getConfigurator().apply(talonFXConfigs.Slot0, 1); // Apply to slot 1


        // kP = 0.5; 
        // kI = 0;
        // kD = 0; 
        // kIz = 0; 
        // kFF = 0.000015; 
        // kMaxOutput = 1; 
        // kMinOutput = -1;

        SmartDashboard.putNumber("kG Gain", kG);

        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);


    
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        Logger.recordOutput("ElevatorPosition Rots", elevatorMotor.getPosition().getValueAsDouble());
    
        Logger.recordOutput("ElevatorPosition Meters", (elevatorMotor.getPosition().getValueAsDouble()/36) * Math.PI * 2 );

        
        Logger.recordOutput("ElevatorPosition Meters with GEAR ratio", (elevatorMotor.getPosition().getValueAsDouble() / 36) * (Math.PI * Units.inchesToMeters(1.5)));
        Logger.recordOutput("ElevatorPosition Meters with GEAR ratio fipped", (elevatorMotor.getPosition().getValueAsDouble() / (1/36) * (2 * Math.PI * Units.inchesToMeters(1.5))));
        Logger.recordOutput("ElevatorPosition closed loop slot", elevatorMotor.getClosedLoopSlot().getValueAsDouble());

        double g = SmartDashboard.getNumber("kG Gain", 0);
    
        // // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((g != kG)) { 
            // config.closedLoop.p(p); 
            kG = g;
            var slot1 = talonFXConfigs.Slot1;

            slot1.kG = kG; // Replace with your determined value
            //elevatorMotor.getConfigurator().apply(slot1, 1); // Apply to slot 1

        } 
        SmartDashboard.putNumber("kG Value", talonFXConfigs.Slot1.kG);





    }

    @Override
    public void setVoltage(double volts) {
        //elevatorMotor.setControl(m_mmReq.withPosition(volts * 12).withSlot(0));
        //elevatorMotor.setVoltage(volts);
        elevatorMotor.getConfigurator().apply(talonFXConfigs.Slot1, 1);
        elevatorMotor.setControl(elevatorRequest.withOutput(volts/2));
    }

    @Override
    public double getHeight(double height) {
        return height;
    }



    public void sysIDTest() {

    }

    @Override
    public void setHeight(double userGoal) {
       // double rots = -(userGoal/(2 *Math.PI)) * /*gear ratio */ 36;
       // double rots = -(userGoal / (2 * Math.PI * Units.inchesToMeters(1.5))) * 36; // Convert meters to rotations

        //elevatorMotor.setPosition(Rotations.of(rots));
        //elevatorMotor.setControl(elevatorRequ.withPosition(rots));
        elevatorMotor.getConfigurator().apply(talonFXConfigs.Slot0, 1);

        elevatorMotor.setControl(m_request.withPosition(userGoal));

        // goal = new TrapezoidProfile.State(userGoal, 0);
        // setpoint = new TrapezoidProfile.State(encoder.getPosition(), 0);
        // setpoint = profile.calculate(1, setpoint, goal);
        // //profile.calculate(5, new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(5, 0));
        // //closedLoopController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward.calculate(setpoint.velocity));
        // closedLoopController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }

    public void zeroElevator() {
        elevatorMotor.setPosition(0);
    }
}