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
  private final String[] limelightNames;
  private final DoubleArrayPublisher[] orientationPublishers;
  private final DoubleSubscriber[] latencySubscribers;
  private final DoubleSubscriber[] txSubscribers;
  private final DoubleSubscriber[] tySubscribers;
  private final DoubleSubscriber[] taSubscribers;
  private final DoubleArraySubscriber[] megatag2Subscribers;
  private final Alert[] disconnectedAlerts;
  private final TargetObservation[] latestTargetObservations;
  private final boolean[] connected;
  private final Integer[] primaryTagIds;

  // GLOBAL
  private final Supplier<Rotation2d> rotationSupplier;
  private final VisionConsumer consumer;

  /**
   * @param consumer         A callback that receives valid vision pose measurements.
   * @param rotationSupplier A supplier for the current robot rotation.
   * @param limelightNames   Varargs of limelight NetworkTable names.
   */
  public VisionSubsystem(VisionConsumer consumer, Supplier<Rotation2d> rotationSupplier, String... limelightNames) {
    this.consumer = consumer;
    this.rotationSupplier = rotationSupplier;
    this.limelightNames = limelightNames;

    int count = limelightNames.length;
    orientationPublishers = new DoubleArrayPublisher[count];
    latencySubscribers = new DoubleSubscriber[count];
    txSubscribers = new DoubleSubscriber[count];
    tySubscribers = new DoubleSubscriber[count];
    taSubscribers = new DoubleSubscriber[count];
    megatag2Subscribers = new DoubleArraySubscriber[count];
    disconnectedAlerts = new Alert[count];
    latestTargetObservations = new TargetObservation[count];
    connected = new boolean[count];
    primaryTagIds = new Integer[count];

    for (int i = 0; i < count; i++) {
      var table = NetworkTableInstance.getDefault().getTable(limelightNames[i]);
      orientationPublishers[i] = table.getDoubleArrayTopic("robot_orientation_set").publish();
      latencySubscribers[i] = table.getDoubleTopic("tl").subscribe(0.0);
      txSubscribers[i] = table.getDoubleTopic("tx").subscribe(0.0);
      tySubscribers[i] = table.getDoubleTopic("ty").subscribe(0.0);
      taSubscribers[i] = table.getDoubleTopic("ta").subscribe(0.0);
      megatag2Subscribers[i] = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
      disconnectedAlerts[i] = new Alert("Vision camera " + limelightNames[i] + " is disconnected.", AlertType.kWarning);
      latestTargetObservations[i] = new TargetObservation(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0));
      connected[i] = false;
      primaryTagIds[i] = null;
    }
  }

  /**
   * Returns the target’s horizontal (tx) angle for the specified limelight.
   *
   * @param index The index of the limelight.
   * @return The tx value (in degrees) as a Rotation2d.
   */
  public Rotation2d getTargetX(int index) {
    return latestTargetObservations[index] != null ? latestTargetObservations[index].tx() : Rotation2d.fromDegrees(0.0);
  }

  public Optional<Integer> getPrimaryTagId(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= primaryTagIds.length) {
        return Optional.empty();
    }
    return Optional.ofNullable(primaryTagIds[cameraIndex]);
}


  @Override
  public void periodic() {
    int count = limelightNames.length;
    // For each limelight, update its connection status, target observation, and publish current orientation.
    for (int i = 0; i < count; i++) {
      // Update connection: if "tl" hasn't updated within 250ms, mark as disconnected.
      connected[i] = ((RobotController.getFPGATime() - latencySubscribers[i].getLastChange()) / 1000) < 250;
      disconnectedAlerts[i].set(!connected[i]);

      // Update target observation (tx and ty)
      double tx = txSubscribers[i].get();
      double ty = tySubscribers[i].get();
      double ta = taSubscribers[i].get();
      latestTargetObservations[i] = new TargetObservation(Rotation2d.fromDegrees(tx), Rotation2d.fromDegrees(ty));

      // Publish current robot orientation (Used for MegaTag2)
      double[] orientation = new double[] { rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 };
      orientationPublishers[i].accept(orientation);
      // LimelightHelpers.SetRobotOrientation(limelightNames[count], rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
    }
    NetworkTableInstance.getDefault().flush();

    // For each limelight, process pose observations from MegaTag2.
    for (int i = 0; i < count; i++) {
      // Read all new samples from the MegaTag2 topic.
      var rawSamples = megatag2Subscribers[i].readQueue();
      Set<Integer> tagIdsSet = new HashSet<>();
      List<PoseObservation> poseObsList = new LinkedList<>();

      Integer bestTagId = null;
      double bestArea = 0;

      for (var rawSample : rawSamples) {
        if (rawSample.value.length == 0) continue;

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
        primaryTagIds[i] = bestTagId;
      } else {
        primaryTagIds[i] = null;
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
        if (i < VisionConstants.cameraStdDevFactors.length) {
          linearStdDev *= VisionConstants.cameraStdDevFactors[i];
          angularStdDev *= VisionConstants.cameraStdDevFactors[i];
        }

        // Send the valid vision measurement to the consumer.
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
        );
      }
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
