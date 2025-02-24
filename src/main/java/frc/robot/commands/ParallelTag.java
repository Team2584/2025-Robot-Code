package frc.robot.commands;

import frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.VisionConstants;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class ParallelTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionSubsystem vision;
  private final Telemetry logger;
  private final Supplier<Double> translationX;
  private final Supplier<Double> translationY;
  private final PIDController pid;
  private final int cameraIndex;

  public ParallelTag(
    CommandSwerveDrivetrain drivetrain,
    VisionSubsystem vision, Telemetry logger,
    Supplier<Double> translationX,
    Supplier<Double> translationY, int cameraIndex) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.logger = logger;
    this.translationX = translationX;
    this.translationY = translationY;
    this.cameraIndex = cameraIndex;

    pid = new PIDController(6, 0.0, 0.0);
    pid.setSetpoint(0.0); // Diff between actual and target heading should be 0
    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    // Get the primary tag ID seen by Camera 0 from the vision subsystem.
    Optional<Integer> maybeTagId = vision.getPrimaryTagId(cameraIndex);
    if (maybeTagId.isEmpty()) {
      // If no tag is detected, default to no rotational override.
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withVelocityX(translationX.get())
              .withVelocityY(translationY.get())
              .withRotationalRate(0.0));
      return;
    }
    int tagId = maybeTagId.get();

    // Look up the known pose of the detected tag.
    Optional<Pose2d> maybeTagPose = VisionConstants.aprilTagLayout.getTagPose(tagId)
        .map(p3d -> p3d.toPose2d());
    if (maybeTagPose.isEmpty()) {
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withVelocityX(translationX.get())
              .withVelocityY(translationY.get())
              .withRotationalRate(0.0));
      return;
    }
    Pose2d tagPose = maybeTagPose.get();

    // Get the robot's current pose from the drivetrain.
    Pose2d currentPose = logger.getPose();

    // Compute the desired heading: the angle from the robot's current position to
    // the tag.
    double desiredRadians = Math.atan2(tagPose.getY() - currentPose.getY(),
        tagPose.getX() - currentPose.getX());
    Rotation2d desiredHeading = Rotation2d.fromRadians(desiredRadians);

    // Compute the error between current heading and desired heading.
    double currentHeadingDegrees = currentPose.getRotation().getDegrees();
    double desiredHeadingDegrees = desiredHeading.getDegrees();
    double error = desiredHeadingDegrees - currentHeadingDegrees;
    // Normalize error to the range [-180, 180] degrees.
    error = ((error + 180) % 360) - 180;

    // Compute a rotation command from the PID controller.
    double rotationCommand = pid.calculate(error);

    // Get the driver’s translation inputs.
    double vx = translationX.get();
    double vy = translationY.get();

    // Command the drivetrain using the driver's translation but overriding
    // rotation.
    drivetrain.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(rotationCommand));
  }

  @Override
  public void end(boolean interrupted) {}
}
