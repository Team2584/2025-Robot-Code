package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Drives relative to a tag offset of the largest tag seen by cameraIndex
 * Limelight.
 * This uses the TAG for horizontal offset and distance to target.
 */

public class DriveTagOffset extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Telemetry logger;
    private final int cameraIndex;

    private double TX;
    private double sideOffset;
    private double TDISTANCE;
    private double forwardOffset;

    private final PIDController PIDx;
    private final PIDController PIDy;
    private final PIDController PIDh;

    public DriveTagOffset(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, Telemetry logger,
            int cameraIndex, double sideOffset, double forwardOffset) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.logger = logger;
        this.cameraIndex = cameraIndex;

        this.sideOffset = sideOffset;
        this.forwardOffset = forwardOffset;

        PIDx = new PIDController(6, 0.0, 0.0);
        PIDx.setSetpoint(0.0);

        PIDy = new PIDController(6, 0.0, 0.0);
        PIDy.setSetpoint(0.0);

        PIDh = new PIDController(3, 0.0, 0.0);
        PIDh.setSetpoint(0.0);

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        TX = vision.getTX(cameraIndex);
        TDISTANCE = Math.sqrt(Math.pow(vision.getDistance(cameraIndex), 2) - Math.pow(TX, 2));

        PIDx.reset();
        PIDy.reset();
        PIDh.reset();
    }

    @Override
    public void execute() {
        // ROTATION
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

        double targetHeadingRad = tagPose.rotateBy(Rotation2d.fromRadians(Math.PI)).getRotation().getRadians();
        double currentHeadingRad = logger.getHeading().getRadians();

        double headingErorr = targetHeadingRad - currentHeadingRad;

        // Normalize error to the range [-180, 180] degrees.
        headingErorr = ((headingErorr + Math.PI) % (2 * Math.PI)) - Math.PI;

        // X & Y
        TX = vision.getTX(cameraIndex);
        TDISTANCE = Math.sqrt(Math.pow(vision.getDistance(cameraIndex), 2) - Math.pow(TX, 2));

        // flip because Y is the side to side of robot; X is forward back of robot
        double velY = PIDx.calculate(sideOffset - TX);
        double velX = PIDy.calculate(TDISTANCE - forwardOffset);
        double rotRate = PIDh.calculate(headingErorr); // NOT SETTING ROTATIONAL RATE (WILL TEST AFTER WORKING)

        drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                        .withVelocityX(velX)
                        .withVelocityY(velY)
                        .withRotationalRate(0)
        // .withRotationalRate(rotRate)
        );
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(TDISTANCE - forwardOffset) < 0.3 && Math.abs(sideOffset - TX) < 6) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }
}