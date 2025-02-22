// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class OIConstants {
        public static int kDriverPort = 0;
        public static int kOperatorPort = 1;
    }

    public class ElevatorConstants {
        public static int kElevatorID = 14;

        public static final double kP = 0.0025;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kMaxVelocity = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kMaxError = 0;

        public static final double kConvertionFactor = (( 16 / 1 ) / 6 + (3 / 4));
        public static final double kStageFour = 165.5;
        public static final double kStageThree = 90;
        public static final double kStageTwo = 32.7675;
        public static final double kStageOne = 5;
        public static final double kMaxHeight = 171;
        public static final double kMinHeight = 0;


    }

    public class FunnelConstants {

    }

    public class ClimberConstants {
        public static final int kClimberMotorID = 0;
    }

    public class CoralManipulatorConstants {

    }

    public class ArmConstants{
        public static final int armMotorId = 14;
        public static final int followingArmMotorID = 62; 

        public static final double kStowAngle = 0;
        public static final double kMaxAngle = 180; // Change if we need a different max angle


        public static final double kP = 0.15;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kMaxRadsPerSecond = 0;
        public static final double kMaxRadsPerSecondPerSecond = 0;
    }
}
