// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOReal;
import frc.robot.subsystems.EndEffector.EndEffectorIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final Drive drive;
    private Vision vision;
    private Elevator elevator;
    private Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    private EndEffector EndEffector;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick climberController = new CommandJoystick(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3),
                    (pose) -> {
                });
                elevator = new Elevator(new ElevatorIOReal());
                EndEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());

                // this.vision = new Vision(
                //     drive,
                //     new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                //     new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation)
                // );
               

                break;
            case SIM:
                this.driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                    new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                this.drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(driveSimulation.getModules()[0]),
                    new ModuleIOSim(driveSimulation.getModules()[1]),
                    new ModuleIOSim(driveSimulation.getModules()[2]),
                    new ModuleIOSim(driveSimulation.getModules()[3]),
                    driveSimulation::setSimulationWorldPose);

                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0,
                        driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1,
                        driveSimulation::getSimulatedDriveTrainPose));
                this.elevator = new Elevator(new ElevatorIOSim() {});
                this.EndEffector = new EndEffector(new EndEffectorIOSim(() -> elevator.getHeight(), driveSimulation) {
                     });

                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {});
                this.vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO(){});
                this.EndEffector = new EndEffector(new EndEffectorIO() {});
            break;
        }



        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive)
        );
        autoChooser.addOption("Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization(drive)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption("Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption("Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // autoChooser.addOption("rightSideAuto",
        //     new PathPlannerAuto("Auto Name", true);
        // );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        drive.setDefaultCommand(DriveCommands.joystickDrive
        (
            drive,
            () -> -MathUtil.applyDeadband(MathUtil.clamp(driver.getLeftY(),-0.5,0.5), 0.1),
            () -> -MathUtil.applyDeadband(MathUtil.clamp(driver.getLeftX(),-0.5,0.5), 0.1),
            () -> -MathUtil.applyDeadband(MathUtil.clamp(-driver.getRightX(),-0.5,0.5), 0.1))
        );

        //Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(
                driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        
        driver.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));


        if (Constants.currentMode == Constants.Mode.REAL) {

            /* Open Loop Control for Elevator */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.2).whileTrue(elevator.runTeleop(() -> operator.getLeftY()));

            operator.povUp().whileTrue(elevator.setHeight(300));
            operator.povLeft().whileTrue(elevator.setHeight(200));
            operator.povRight().whileTrue(elevator.setHeight(100));
            operator.povDown().whileTrue(elevator.setHeight(0));

            EndEffector.setDefaultCommand(EndEffector.runTeleop(() -> operator.getLeftTriggerAxis()/4, ()-> operator.getRightTriggerAxis()/4, () -> operator.getLeftY()));

            climberController.axisMagnitudeGreaterThan(Joystick.AxisType.kY.value, 0.2)
            .and(climberController.button(1))
            .whileTrue(climber.runTeleop(() -> climberController.getY()))
            .onFalse(climber.runTeleop(() -> 0));


        }         
        else if (Constants.currentMode == Constants.Mode.SIM) {
            // PathConstraints constraints = new PathConstraints(3.0, 4.0,
            //     Units.degreesToRadians(540), Units.degreesToRadians(720)
            // );
            // try { controller.b().whileTrue(
            //     AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("toReed"),constraints));
            // } 
            // catch (FileVersionException | IOException | ParseException e) {e.printStackTrace();}

            // driver.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1)
            // .whileTrue(claw.moveElevatorUpCommand())
            // .onFalse(claw.holdElevatorPositionCommand());

            
            
            
            // driver.leftTrigger().whileTrue(getAutonomousCommand());
    
        }

    }
    

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }


    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


}
