package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Swerve.ModuleIO.ModuleIOInputs;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputs inputs = new ModuleIOInputs();
  private final int index;
  private double absoluteEncoderOffsetRad;
  private boolean absoluteEncoderReversed;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  /* ------ */
    public void updateInputs() {
        io.updateInputs(inputs);
        // Logger.processInputs("Drive/Module" + Integer.toString(index), inputs); 

        // int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        // odometryPositions = new SwerveModulePosition[sampleCount];
        // for (int i = 0; i < sampleCount; i++) {
        //     double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
        //     Rotation2d angle = inputs.odometryTurnPositions[i];
        //     odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        // }
    }
  
    public void resetEncoders() {
        io.resetEncoders();
    }

    public void setDesiredState(SwerveModuleState state) {
        io.setDesiredState(state);
    }

    public void stop() {
        io.stop();
    }

    public void brake(boolean doBrake){
      io.brake(doBrake); 
    }


}