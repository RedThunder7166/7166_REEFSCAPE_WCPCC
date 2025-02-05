// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static final String CANIVORE_NAME = "HighVoltageCANivore";

    public static final int CANDLE_ID = 20;
    public static final int CANDLE_LED_COUNT = 47;

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class ElevatorConstants {
        public static final int LEADER_MOTOR_ID = 17; // left
        public static final int FOLLOWER_MOTOR_ID = 12; // right

        public static final double MIN_POSITION_ROTATIONS = 0.01; // essentially the error, since ideal min position is 0
        public static final double MAX_POSITION_ROTATIONS = 0.5;

        // TODO: ideally, home and coral station are the same
        // FIXME: ELEVATOR HOME POSITION
        public static final double HOME_POSITION = 0;
        // FIXME: ELEVATOR CORAL STATION POSITION
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        // FIXME: ELEVATOR L1 POSITION
        public static final double L1_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L2 POSITION
        public static final double L2_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L3 POSITION
        public static final double L3_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L4 POSITION
        public static final double L4_POSITION = HOME_POSITION;

        // FIXME: ELEVATOR MANUAL VELOCITY
        public static final double MANUAL_VELOCITY_FORWARD_RPS = 1;
        public static final double MANUAL_VELOCITY_BACKWARD_RPS = -MANUAL_VELOCITY_FORWARD_RPS;
    }
    public static final class ClawConstants {
        // FIXME: CLAW INTAKE_MOTOR ID
        public static final int INTAKE_MOTOR_ID = 16;
        // FIXME: CLAW WRIST_MOTOR ID
        public static final int WRIST_MOTOR_ID = 9;
        // FIXME: CLAW CANCODER ID
        public static final int CANCODER_ID = 26;

        // FIXME: INTAKE VELOCITY
        public static final double INTAKE_FORWARD_VELOCITY_RPS = 1;
        public static final double INTAKE_BACKWARD_VELOCITY_RPS = -INTAKE_FORWARD_VELOCITY_RPS;

        // FIXME: CLAW WRIST CORAL STATION POSITION
        public static final double WRIST_CORAL_STATION_POSITION = 0;

        // FIXME: CLAW WRIST L1 POSITION
        public static final double WRIST_L1_POSITION = WRIST_CORAL_STATION_POSITION;
        // FIXME: CLAW WRIST L2-L3 POSITION
        public static final double WRIST_L2_3_POSITION = WRIST_CORAL_STATION_POSITION;
        // FIXME: CLAW WRIST L4 POSITION
        public static final double WRIST_L4_POSITION = WRIST_CORAL_STATION_POSITION;

        public static final double WRIST_MIN_POSITION = 1; // ideally 0
        public static final double WRIST_MAX_POSITION = 160;

        // FIXME: WRIST MANUAL VELOCITY
        public static final double MANUAL_WRIST_VELOCITY_FORWARD_RPS = 3;
        public static final double MANUAL_WRIST_VELOCITY_BACKWARD_RPS = -MANUAL_WRIST_VELOCITY_FORWARD_RPS;
    }
}
