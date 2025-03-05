package frc.robot.subsystems.Algae;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private final AlgaeIO io;
    private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
    public double elevatorHeight = 0;
    //private final SysIdRoutine sysId;

    public Algae(AlgaeIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
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