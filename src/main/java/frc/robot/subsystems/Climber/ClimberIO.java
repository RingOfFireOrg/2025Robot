package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;


public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}
  public default void setVoltage(double volts) {}

}