package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double controllerSetpoint = 0.0;
    public double controllerOutput = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double angularPositionRot = 0.0;
    public double elevatorPositionMeters = 0.0;
    public Pose3d elevatorPose3d = new Pose3d();

  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default double getHeight(double height) {return height;}
  public default void setHeight(double height) {}

}