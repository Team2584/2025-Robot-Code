package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Drives relative to a tag offset of the largest tag seen by cameraIndex
 * Limelight.
 * This uses the global POSE ESTIMATION of the robot
 */

public class DriveRelativeTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Telemetry logger;

    PathPlannerPath path;

    private final Translation2d relativeTagVector; // A pose2d that is referenced off the tag (ID'd from
                                                   // vision.getPrimaryTagId(cameraIndex))
    private final int cameraIndex; // The camera being used

    private FollowPathCommand followCommand;

    private double distanceToTarget;
    Pose2d targetPose;

    private final PPHolonomicDriveController holonomicController;

    public DriveRelativeTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, Telemetry logger,
            Translation2d relativeTagVector, int cameraIndex) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.logger = logger;
        this.relativeTagVector = relativeTagVector;
        this.cameraIndex = cameraIndex;

        holonomicController = new PPHolonomicDriveController(
                new PIDConstants(10, 0, 0), // TRANSLATION PID
                new PIDConstants(7, 0, 0) // ROTATION PID
        );

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        var maybeTagId = vision.getPrimaryTagId(cameraIndex);
        if (maybeTagId.isEmpty()) {
            // If no tag is detected, cancel the command.
            cancel();
            return;
        }

        int tagId = maybeTagId.get();

        // Look up the known pose of that tag from the field layout.
        var maybeTagPose = VisionConstants.aprilTagLayout.getTagPose(tagId).map(p3d -> p3d.toPose2d());
        if (maybeTagPose.isEmpty()) {
            cancel();
            return;
        }

        Pose2d tagPose = maybeTagPose.get();

        // Calculate target pose (using relativeTagVector)
        targetPose = new Pose2d(
                new Translation2d(
                        tagPose.getTranslation().getX() +
                                relativeTagVector.getX() * Math.cos(tagPose.getRotation().getRadians()) -
                                relativeTagVector.getY() * Math.sin(tagPose.getRotation().getRadians()),

                        tagPose.getTranslation().getY() +
                                relativeTagVector.getY() * Math.cos(tagPose.getRotation().getRadians()) +
                                relativeTagVector.getX() * Math.sin(tagPose.getRotation().getRadians())),
                tagPose.getRotation().rotateBy(new Rotation2d(Math.PI)));

        // Log TargetPose
        SmartDashboard.putNumber("TargetPoseX", targetPose.getX());
        SmartDashboard.putNumber("TargetPoseY", targetPose.getY());

        // Get the robot's starting pose.
        Pose2d startPose = new Pose2d(logger.getPose().getTranslation(), logger.getPose().getRotation());

        // PATH CALCS

        Rotation2d travelDirection = Rotation2d.fromRadians(
                Math.atan2(targetPose.getY() - startPose.getY(), targetPose.getX() - startPose.getX()));
        Pose2d startWaypoint = new Pose2d(startPose.getTranslation(), travelDirection);
        Pose2d endWaypoint = new Pose2d(targetPose.getTranslation(), travelDirection);

        distanceToTarget = targetPose.getTranslation().getDistance(startPose.getTranslation());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startWaypoint, endWaypoint);

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, targetPose.getRotation()));

        path.preventFlipping = true;

        try {
            var config = RobotConfig.fromGUISettings();
            followCommand = new FollowPathCommand(
                    path,
                    logger::getPose,
                    drivetrain::getRobotRelativeSpeeds,
                    drivetrain.getDriveOutput(),
                    holonomicController,
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    drivetrain);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure FollowPathCommand",
                    ex.getStackTrace());
        }

        followCommand.initialize();
    }

    @Override
    public void execute() {
        distanceToTarget = targetPose.getTranslation().getDistance(logger.getPose().getTranslation());
        followCommand.execute();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(distanceToTarget) < 0.05) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        followCommand.end(interrupted);
        drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}