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

import edu.wpi.first.wpilibj.RobotBase;


public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    /*
     * 20 algae pivor
     * 21 left algae spin
     * 22 right algae spin
     */

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }
    public static enum EndEffectorPositionState {
        STOWED,
        INTAKE,
        SCORE,
        SCOREL4
    }
    public static enum EndEffectorIntakeState {
        INTAKE,
        EJECT,
        HOLD,
        NONE
    }

    public static class OIConstants {
        public static final double controllerDeadband = 0.1;
    }

    public static class ElevatorHeights {
        public static final double OFF_THE_GROUND = 0;
        public static final double STOWED = 0;
        public static final double INTAKE_HEIGHT = 0.89;
        public static final double L2 = .993;

        public static final double L3 = 3.1;
        public static final double L4 = 0;



    }
    public static class PivotAngles {
        public static final double STOWED = 0.37;
        public static final double INTAKE = 0.39;
        public static final double L2 = 0.57;
        public static final double L3 = 0.5;
        public static final double ALGAE = 0.36;




        public static final double Intake_Coral = 0.7;
        public static final double Eject_Coral = -0.7;
        public static final double Maintain_Coral = 0.3;

    }

    public static class AlgaeAngles {
        public static final double STOWED = 0.41;
        public static final double LOWER_ALGAE = 0.6;
        public static final double UPPER_ALGAE = 0.47;


    }
}
