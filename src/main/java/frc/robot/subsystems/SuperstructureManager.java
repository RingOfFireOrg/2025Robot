package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.EndEffector;

public class SuperstructureManager {
    private final Elevator elevator;
    private final EndEffector endEffector;

    public enum SuperstructureState {
        STOWED, INTAKING, SCORING, EJECTING,


        PREP_L2,
        PREP_L3,
        PREP_L4,
        MANUAL_CONTROL
    }

    private SuperstructureState currentState = SuperstructureState.STOWED;

    public SuperstructureManager(Elevator elevator, EndEffector endEffector) {
        this.elevator = elevator;
        this.endEffector = endEffector;
    }

    public void setState(SuperstructureState newState) {
        this.currentState = newState;
    }

    public void update() {
        switch (currentState) {
            case STOWED:
                // elevator.setHeight(0);
                // endEffector.setPivotAngle(0);
                // endEffector.setIntakeSpeed(0);
                break;
            
            case INTAKING:
            //     endEffector.setPivotAngle(30); // Lower pivot for intake
            //     elevator.setHeight(0);  // Keep elevator low
            //     endEffector.setIntakeSpeed(1); // Spin intake
                break;
            
            case SCORING:
                // elevator.setHeight(50); // Raise elevator
                // endEffector.setPivotAngle(60); // Adjust pivot for scoring
                // endEffector.setIntakeSpeed(0); // Stop intake
                break;

            case EJECTING:
                // endEffector.setPivotAngle(60);
                // endEffector.setIntakeSpeed(-1); // Reverse intake to eject
                break;
        }
    }

    public Command returnState(SuperstructureState stateRequest) {
        switch (currentState) {
            case STOWED:
            
            case PREP_L4:
            // case PREP_L3_ALGAE:
                // Stow Intake
            case PREP_L3:
                //if Algae is up, spin till stall
            case PREP_L2:
                //Check if Algae is lowered
            // case PREP_INTAKE:
            case INTAKING:
            // case INTAKING_HOLD:
        }
        return new InstantCommand();
    }


    public SuperstructureState getState() {
        return currentState;
    }
}

