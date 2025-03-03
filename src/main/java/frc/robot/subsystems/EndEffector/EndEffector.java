package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorIOInputsAutoLogged;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;

  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  double angle;
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("End Effector", inputs);
    angle = inputs.angularPosition;
  }

  public Command runPercent(double percent) {
    return runEnd(() -> io.setVoltage(percent * 12.0), () -> io.setVoltage(0.0));
  }

  public Command angle(double angle) {
    return run(() -> io.setAngle(angle));
  }

  public Command runTeleop(DoubleSupplier forward, DoubleSupplier reverse, DoubleSupplier eject) {
    return runEnd(
      () -> io.setVoltageEject((forward.getAsDouble() - reverse.getAsDouble()) * 12.0, eject.getAsDouble() * 12.0),
      () -> io.setVoltageEject(0.0,0.0)
    );
  }

  public double getAngle() {
    return angle;
  }
}