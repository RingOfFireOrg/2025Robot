package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class LEDStatusCMD extends Command {
  LEDSubsystem ledSubsystem;
  private boolean redAlliance;
  public LEDStatusCMD(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);
    this.ledSubsystem = ledSubsystem;  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    redAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    if (DriverStation.isDisabled()) {

    }
    else if (DriverStation.isAutonomousEnabled()) {

    }
    //else if () {}
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
