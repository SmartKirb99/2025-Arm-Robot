package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class DriveConstants {
    public static final int kLeftMasterPort = 0;
    public static final int kRightMasterPort = 3;
    public static final int kLeftFollowerPort = 1;
    public static final int kRightFollowerPort = 2;
    public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;

    public static final double kAutoDriveForwardSpeed = 0.25;
    public static final double kAutoDriveForwardDistance = 1.5;
  }

  public static final class OIConstants {
    public static final int kDriverJoystickPort = 0;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 3;

    public static int kOperatorController = 1;
  }

  public static final class LimelightConstants {

	public static double kGoalHeightMeters = Units.inchesToMeters(57.88);
    public static double kLimelightLensHeightMeters = 0.2;
    public static double kMountAngleRadians = Units.degreesToRadians(0);

  }




// Yes I changed the "k" in the original code into an "r" 1, because funny, and 2 , rams, instead of kangaroos or whatever the "k" was supposed to symbolize
// I suppose it could also be the funny subreddit symbol or something like that
  public static final class ArmConstants{
    public static final int armMotorId =  13 ;

    public static final double rStowAngle = 0;
    public static final double rStage1Angle = 170;
    public static final double rStage2Angle = 32.8;
    public static final double rStage3Angle = 80;
    public static final double rHumanStageAngle = 22;
    public static final double rDropStageAngle = 38.7;
    public static final double rTippedCoralAngle = 159.52;
    public static final double rMaxAngle = 180;
    public static final double rClawHeightOffset = 0.3;
    public static final double rClawLengthOffset = 0.25;
    public static final double rArmLength = 1;
   
   
   // I decided to keep these the same as they were
    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV  =  0;
    public static final double kA = 0;

    public static final double kMaxRadsPerSecond = 0; // Uhh, I don't think we're supposed to be messing with Rads
    public static final double kMaxRadsPerSecondPerSeond = 0;
  }


}