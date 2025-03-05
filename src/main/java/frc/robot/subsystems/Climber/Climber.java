package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    public double elevatorHeight = 0;
    //private final SysIdRoutine sysId;

    public Climber(ClimberIO io) {
        this.io = io;
        // sysId = new SysIdRoutine(
        //     new SysIdRoutine.Config(null, null, null,
        //         (state) -> Logger.recordOutput("ElevatorSysID", state.toString())),
        //     new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(BaseUnits.VoltageUnit)), null,
        //         this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        elevatorHeight = inputs.elevatorPositionMeters;

    }

    public Command runPercent(double percent) {
        return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
    }

    public Command runTeleop(DoubleSupplier input) {
        return run(() -> io.setVoltage((input.getAsDouble()) * 12.0));
    }

    //TODO: remove
    // public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
    //     return runEnd(
    //         () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
    //         () -> io.setVoltage(0.0));
    // }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    // }

    // /** Returns a command to run a dynamic test in the specified direction. */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    // }

    public Command setHeight(double height) {
        return run(() -> io.setHeight(height));
    }

    public double getHeight() {
        return elevatorHeight;
    }

    // public void runCharacterization(double output) {
    //     io.setDriveOpenLoop(output);
    // }
}