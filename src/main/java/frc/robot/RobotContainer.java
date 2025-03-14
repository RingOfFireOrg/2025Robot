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

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorHeights;
import frc.robot.Constants.PivotAngles;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToCurrentReef;
import frc.robot.subsystems.Algae.Algae;
import frc.robot.subsystems.Algae.AlgaeIOReal;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIORealTalon;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOReal;
import frc.robot.subsystems.EndEffector.EndEffectorIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

public class RobotContainer {
    private final Drive drive;
    private Vision vision;
    private Elevator elevator;
    private Climber climber;
    private SwerveDriveSimulation driveSimulation = null;
    private EndEffector EndEffector;
    private Algae algae;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final CommandJoystick climberController = new CommandJoystick(2);
    private final CommandXboxController spareTest = new CommandXboxController(4);

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
                elevator = new Elevator(new ElevatorIORealTalon());
                EndEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                algae = new Algae(new AlgaeIOReal());
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


        setNamedCommands();

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
        // autoChooser.addOption("leftOnlyPathing",
        //     drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        // );
        
        // autoChooser.addOption("rightSideAuto",
        //     new PathPlannerAuto("Auto Name", true);
        // );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        double maxSpeed = 0.6;
        // drive.setDefaultCommand(DriveCommands.joystickDrive(
        //     drive,
        //     () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(),-maxSpeed,maxSpeed), 0.1),
        //     () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(),-maxSpeed,maxSpeed), 0.1),
        //     () -> MathUtil.applyDeadband(MathUtil.clamp(driver.getRightX(),-maxSpeed,maxSpeed), 0.1))
        // );

        double standardSpeed = 0.5;
        drive.setDefaultCommand(DriveCommands.joystickDrive(
         drive,
            () -> {
                double maxSpeedX = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(), -maxSpeedX, maxSpeedX), 0.1);
            },
            () -> {
                double maxSpeedY = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(), -maxSpeedY, maxSpeedY), 0.1);
            },
            () -> {
                double maxSpeedTheta = (1 - driver.getLeftTriggerAxis()) * (standardSpeed + (1-standardSpeed) * driver.getRightTriggerAxis());
                return MathUtil.applyDeadband(MathUtil.clamp(-driver.getRightX(), -maxSpeedTheta, maxSpeedTheta), 0.1);
            }
        ));


        driver.povRight().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> -0.4, () -> 0));
        driver.povLeft().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0, () -> 0.4, () -> 0));

        driver.povUp().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> 0.4, () -> 0, () -> 0));
        driver.povDown().whileTrue(DriveCommands.joystickDriveRobotOriented(drive, () -> -0.4, () -> 0, () -> 0));

        driver.rightBumper().whileTrue(DriveCommands.joystickDriveAtAngle(drive, 
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(),-maxSpeed,maxSpeed), 0.1),
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(),-maxSpeed,maxSpeed), 0.1),
        () -> new Rotation2d(-125) ));

        driver.leftBumper().whileTrue(DriveCommands.joystickDriveAtAngle(drive, 
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftY(),-maxSpeed,maxSpeed), 0.1),
        () -> MathUtil.applyDeadband(MathUtil.clamp(-driver.getLeftX(),-maxSpeed,maxSpeed), 0.1),
        () -> new Rotation2d(125) ));


        //driver.a().whileTrue(new DriveToCurrentReef(drive, vision));

        //Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(
                driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        
        driver.back().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));


        if (Constants.currentMode == Constants.Mode.REAL) {


            operator.y()
            .whileTrue(algae.runTeleop(() -> -0.4))
            .onFalse(algae.runTeleop(() -> 0.0));
            operator.a()
            .whileTrue(algae.runTeleop(() -> 0.4))
            .onFalse(algae.runTeleop(() -> 0.0));

            operator.x()
            .whileTrue(algae.runTeleopIntake(() -> 0.7))
            .onFalse(algae.runTeleopIntake(() -> 0.0));
            operator.b()
            .whileTrue(algae.runTeleopIntake(() -> -0.7))
            .onFalse(algae.runTeleopIntake(() -> 0.0));

            operator.povUp()
            .whileTrue(elevator.setHeight(ElevatorHeights.L3))
            .onTrue(EndEffector.angle(PivotAngles.L3))
            ;
            operator.povLeft()
            .whileTrue(elevator.setHeight(ElevatorHeights.L2))
            .onTrue(EndEffector.angle(PivotAngles.L2))
            ;
            operator.povRight()
            .whileTrue(elevator.setHeight(ElevatorHeights.L2))
            .onTrue(EndEffector.angle(PivotAngles.L2))
            ;

            operator.povDown()
            .whileTrue(elevator.setHeight(0))
            .onTrue(EndEffector.angle(PivotAngles.STOWED))
           ;

    

            // operator.y().onTrue(EndEffector.angle(PivotAngles.STOWED));
            // operator.x().onTrue(EndEffector.angle(0.45));
            // operator.a().onTrue(EndEffector.angle(PivotAngles.STANDARD_CORAL));

            /* Intake Coral */
            operator.leftBumper()
            .onTrue(EndEffector.ejecter(0.7))
            .onFalse(EndEffector.ejecter(0));

            /* Intake Coral */
            operator.rightBumper()
            .onTrue(EndEffector.ejecter(-0.7))
            .onFalse(EndEffector.ejecter(0));

            /* Intaking Setup Button - Move elevator to intake height, move intake to intake angle, and Intake coral */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1)
            .whileTrue(elevator.setHeight(ElevatorHeights.INTAKE_HEIGHT))
            .onTrue(EndEffector.setAngleandIntake(PivotAngles.INTAKE, 0.7))
            .onFalse(EndEffector.ejecter(0))
            ;




            /* Manual Control for ELevator */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
            .whileTrue(elevator.runTeleop(() -> -operator.getLeftY()/1.4))
            .onFalse(elevator.runTeleop(() -> 0))
            ;

            /* Manual Control for Pivot intake */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1)
            .whileTrue(EndEffector.runTeleop(() -> -operator.getRightY()/3, ()-> 0, () -> 0))
            .onFalse(EndEffector.runTeleop(() -> 0, ()-> 0, () -> 0));

            
            /* Eject Coral CMD */
            operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightTrigger.value, 0.1)
            .whileTrue(EndEffector.runTeleop(() -> 0, ()-> 0, () -> -operator.getRightTriggerAxis()))
            .onFalse(EndEffector.runTeleop(() -> 0, ()-> 0, () -> 0));
            //EndEffector.setDefaultCommand(EndEffector.runTeleop(() -> operator.getLeftTriggerAxis()/4, ()-> operator.getRightTriggerAxis()/4, () -> operator.getLeftY()));



            /* Climbing Controls */
            climberController.axisMagnitudeGreaterThan(Joystick.AxisType.kY.value, 0.2)
            .and(climberController.button(1))
            .whileTrue(climber.runTeleop(() -> -climberController.getY()))
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

    public void setNamedCommands() {
        /*
         * Raise Elevator to L3
         * Pivot Endeffector to L3
         */
        NamedCommands.registerCommand("Prep_L3", 
        elevator.runOnceHeight(ElevatorHeights.L3)
        .alongWith(EndEffector.angle(PivotAngles.L3))
        .alongWith(Commands.print("Prep_L3"))
        );
        
        /*
         * Raise Elevator to L2
         * Pivot Endeffector to L2
         */
        NamedCommands.registerCommand("Prep_L2", 
        elevator.runOnceHeight(.92)
        .alongWith(EndEffector.angle(0.5))
        .alongWith(Commands.print("NamedCommand: Prep_L2"))
        );

        /*
         * Raise Elevator to Intake Height
         * Pivot Endeffector to L3
         * Start Intaking
         */
        NamedCommands.registerCommand("Intake", 
        elevator.runOnceHeight(ElevatorHeights.INTAKE_HEIGHT)
        .alongWith(EndEffector.angle(PivotAngles.INTAKE))
        .andThen(EndEffector.ejecter(0.7))
        .alongWith(Commands.print("NamedCommand: Intake"))
        );
        

        NamedCommands.registerCommand("Reset",
        elevator.runOnceHeight(ElevatorHeights.STOWED)
        .andThen(EndEffector.angle(PivotAngles.STOWED))
        .andThen(EndEffector.ejecter(PivotAngles.Maintain_Coral))
        .alongWith(Commands.print("NamedCommand: Reset"))
        ); 

        NamedCommands.registerCommand("Ejecter_Maintain",
        new InstantCommand()
        .alongWith(Commands.print("NamedCommand: Ejecter_Maintain"))
        ); 

        NamedCommands.registerCommand("Ejecter_Eject",
        EndEffector.ejecter(-0.7)
        .alongWith(Commands.print("NamedCommand: Ejecter_Eject"))
        ); 

    }
    
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public EndEffector getEffector() {
        return EndEffector;
    }























    /* -------------------------------------------------------------------------------------------------- */
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




}
