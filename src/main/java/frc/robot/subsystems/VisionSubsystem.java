package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

  // PER-LIMELIGHT
  private final DoubleArrayPublisher orientationPublishers;
  private final DoubleSubscriber latencySubscribers;
  private final DoubleSubscriber txSubscribers;
  private final DoubleSubscriber tySubscribers;
  private final DoubleSubscriber taSubscribers;
  private final DoubleSubscriber tidSubscriber;
  private final DoubleArraySubscriber megatag2Subscribers;
  private final Alert disconnectedAlerts;
  private  TargetObservation latestTargetObservations;
  private boolean connected;
  private Integer primaryTagIds;
  private double MainTagDistance;

  // GLOBAL
  private final Supplier<Rotation2d> rotationSupplier;
  private final VisionConsumer consumer;

  /**
   * @param consumer         A callback that receives valid vision pose measurements.
   * @param rotationSupplier A supplier for the current robot rotation.
   * @param limelightNames   Varargs of limelight NetworkTable names.
   */
  public VisionSubsystem(VisionConsumer consumer, Supplier<Rotation2d> rotationSupplier) {
    this.consumer = consumer;
    this.rotationSupplier = rotationSupplier;



    var table = NetworkTableInstance.getDefault().getTable(VisionConstants.camera0Name);
    tidSubscriber = table.getDoubleTopic("tid").subscribe(0.0);
    orientationPublishers = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscribers = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscribers = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscribers = table.getDoubleTopic("ty").subscribe(0.0);
    taSubscribers = table.getDoubleTopic("ta").subscribe(0.0);
    megatag2Subscribers = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    disconnectedAlerts = new Alert("Vision camera " + VisionConstants.camera0Name + " is disconnected.", AlertType.kWarning);
    latestTargetObservations = new TargetObservation(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));
    connected = false;
    primaryTagIds = null;
    
  }

  /**
   * Returns the target’s horizontal (tx) angle for the specified limelight.
   *
   * @param index The index of the limelight.
   * @return The tx value (in degrees) as a Rotation2d.
   */
  public Rotation2d getTargetX() {
    return latestTargetObservations != null ? latestTargetObservations.tx() : Rotation2d.fromDegrees(0.0);
  }

  public Optional<Double> getPrimaryTagId() {
    if (tidSubscriber.get()==0){
      return Optional.ofNullable(null);
    }
    else{
      return Optional.ofNullable(tidSubscriber.get());
    }

  }

  public double getTX(){
    return txSubscribers.getAsDouble();
  }

  public double getTY(){
    return tySubscribers.getAsDouble();
  }

  public double getDistance(){
    return MainTagDistance;
  }


  @Override
  public void periodic() {
    // Update its connection status, target observation, and publish current orientation.
    // Update connection: if "tl" hasn't updated within 250ms, mark as disconnected.
    connected = ((RobotController.getFPGATime() - latencySubscribers.getLastChange()) / 1000) < 250;
    disconnectedAlerts.set(!connected);

    // Update target observation (tx and ty)
    double tx = txSubscribers.get();
    double ty = tySubscribers.get();
    double ta = taSubscribers.get();
    latestTargetObservations = new TargetObservation(Rotation2d.fromDegrees(tx), Rotation2d.fromDegrees(ty));

    // Publish current robot orientation (Used for MegaTag2)
    double[] orientation = new double[] { rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 };
    orientationPublishers.accept(orientation);
    // LimelightHelpers.SetRobotOrientation(limelightNames[count], rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
    
    NetworkTableInstance.getDefault().flush();

    // Process pose observations from MegaTag2.

    // Read all new samples from the MegaTag2 topic.
    var rawSamples = megatag2Subscribers.readQueue();
    Set<Integer> tagIdsSet = new HashSet<>();
    List<PoseObservation> poseObsList = new LinkedList<>();


    Integer bestTagId = null;
    double bestArea = 0;

    for (var rawSample : rawSamples) {
      if (rawSample.value.length == 0) continue;

      MainTagDistance = rawSample.value[9];


      // Accumulate tag IDs
      for (int j = 11; j < rawSample.value.length; j += 7) {
        tagIdsSet.add((int) rawSample.value[j]);         

        int tagId = (int) rawSample.value[j];
        double area = rawSample.value[j - 1]; // TA is assumed to be at offset 1 in the block.
        if (area > bestArea) {
          bestArea = area;
          bestTagId = tagId;
        }
      }

      // Compute vision timestamp (rawSample.timestamp is in microseconds; rawSample.value[6] is latency in ms)
      double visionTimestamp = rawSample.timestamp * 1e-6 - rawSample.value[6] * 1e-3;
      // Parse raw 3D pose from the first six elements.
      Pose3d rawPose = parsePose(rawSample.value);

      poseObsList.add(new PoseObservation(
          visionTimestamp,              // seconds
          rawPose,
          0.0,                // ambiguity is 0 (MegaTag2)
          (int) rawSample.value[7],
          rawSample.value[9]
      ));
    }

    if (bestTagId != null) {
      primaryTagIds = bestTagId;
    } else {
      primaryTagIds = null;
    }

    // Process each pose observation.
    for (PoseObservation observation : poseObsList) {
      // Filtering criteria: reject if no tags, high ambiguity (for one tag), unrealistic Z, or out-of-bounds.
      boolean rejectPose = (observation.tagCount() == 0)
          || (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity)
          || (Math.abs(observation.pose().getZ()) > VisionConstants.maxZError)
          || (observation.pose().getX() < 0.0 || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength())
          || (observation.pose().getY() < 0.0 || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth());
      if (rejectPose) continue;

      // Compute measurement uncertainties.
      double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor * VisionConstants.linearStdDevMegatag2Factor;
      double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor * VisionConstants.angularStdDevMegatag2Factor;
      

      // Send the valid vision measurement to the consumer.
      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
      );
    }
  }

  /**
   * Parses a Pose3d from raw limelight botpose array.
   *
   * @param rawLLArray The raw array from NetworkTables.
   * @return A Pose3d built from the first six values.
   */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])
        )
    );
  }

  /** Functional interface for receiving valid vision pose measurements. */
  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Record representing a simple target observation (tx/ty). */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Record representing a vision pose observation from MegaTag2. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance
  ) {}
}