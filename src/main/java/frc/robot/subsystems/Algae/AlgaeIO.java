package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
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

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double volts) {}
  public default void setVoltageIntake(double volts) {}

  public default double getHeight(double height) {return height;}
  public default void setHeight(double height) {}

}