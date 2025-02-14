// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;


public class Swerve extends SubsystemBase {
  private final Module[] swerveModules;
  public Swerve(ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    swerveModules = new Module[] {
      new Module(flModuleIO, 0),
      new Module(frModuleIO, 1),
      new Module(blModuleIO, 2),
      new Module(brModuleIO, 3),


      
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
