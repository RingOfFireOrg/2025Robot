package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  public double elevatorHeight = 0;
  private final SysIdRoutine sysId;


  public Elevator(ElevatorIO io) {
    this.io = io;
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("ElevatorSysID", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(BaseUnits.VoltageUnit)), null, this)
    );
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

    public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
        return runEnd(
        () -> io.setVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setVoltage(0.0));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }
    
    public Command setHeight(double height) {
    return run(() -> io.setHeight(height));
    }

    public double getHeight() {
    return elevatorHeight;
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
    }
}