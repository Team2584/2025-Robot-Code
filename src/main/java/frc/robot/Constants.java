package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {

  public static final class ElevatorConstants{
    /* CAN IDs */
  public static final int ELEVATOR_LEFT_ID = 21;
  public static final int ELEVATOR_RIGHT_ID = 20;

  /* Elevator Setpoints, Speeds, and Positions  */
  public static final double HOME = 0; //TUNE ME

  public static final double INTAKE_CORAL = 0; //TUNE ME

  public static final double L1 = 3.8; //TUNE ME
  public static final double L2 = 15; //TUNE ME
  public static final double L3 = 30; //TUNE ME
  public static final double L4 = 54; //TUNE ME

  public static final double GROUND_ALGAE = 0; //TUNE ME
  public static final double ALGAE_LOW = 23; //TUNE ME
  public static final double ALGAE_HIGH = 40; //TUNE ME

  public static final double NET = 54;



  /* Physical Measurements */
  public static final double ELEVATOR_MIN_HEIGHT = 0;
  public static final double ELEVATOR_MAX_HEIGHT = 55; // (actually ~~65in)
  public static final double ELEVATOR_PULLEY_PITCH_DIAMETER  = 1.504;
  public static final double ELEVATOR_GEAR_RATIO = 8.571;
  }

  public static final class WristConstants{

    public static final int WRIST_ID = 16;
    public static final double WRIST_GEAR_RATIO = 279.27;

    public static final int WRIST_ENCODER_ID = 25;
  
    public static final int currentLimit = 60;
  }

  public static final class AlgaeConstants{
    public static final int ALGAE_ID = 17;
  }

  public static final class ClimberConstants{
    public static final int leftClimberCanId = 15;   
     public static final double motorReduction = 180.0;
    public static final int currentLimit = 40; 
  }

  public static final class CoralMechConstants{
    public static final int CORAL_MECH_ID = 23;
    public static final int CORAL_SENSOR_ID = 26;
  }

  public static final class RampConstants{
    public static final int RAMP_ID = 28;
    public static final double RAMP_GEAR_RATIO = 45;
  }
  public static final class VisionConstants {
    // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Camera names, must match names configured
  public static String camera0Name = "limelight-front";
  public static String camera1Name = "limelight-one";

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; 
  }
}
