// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int joystick1Port = 0;
    
    //placeholders, in meters
    public static class ArmConstants {
        public static final double SIM_WIDTH = 5;
        public static final double SIM_HEIGHT = 5;
        public static final double PIVOT_X_OFFSET = 3;
        public static final double PIVOT_Y_OFFSET = 0;
        public static final double INIT_ELEVATOR_LENGTH = 0.1;

        public static final double kElevatorGearing = 10.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kCarriageMass = 4.0; // kg

        public static final double kSetpointMeters = 0.75;
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = 1.25;

        public static final int controller1port = 0;
        public static final int controller2port = 1;

        public static final double kElevatorKp = 5;
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0;
      
        public static final double kElevatorkS = 0.0; // volts (V)
        public static final double kElevatorkG = 0.762; // volts (V)
        public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
        public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

        public static final double maxVelElevator = 2.45; //m/s
        public static final double maxAccelElevator = 2.45; //m/s²
    }
}
