// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.jni.HardwareJNI.Context;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Controller imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RampSubsystem;
import frc.robot.subsystems.USBCameraSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import frc.robot.commands.*;
import frc.robot.Constants.VisionConstants;
import frc.robot.commandgroup.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  private double governor = 0.35; // Added to slow MaxSpeed to 35%. Set to 1 for full speed.
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1 * Math.PI).in(RadiansPerSecond); // 1/2 of a rotation per
                                                                                           // second max angular
                                                                                           // velocity

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Joystick buttonBoard = new Joystick(2);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final Field2d m_field = new Field2d();

  private ElevatorSubsystem buildElevatorSubsystem() {
    return new ElevatorSubsystem();
  }

  private AlgaeSubsystem buildAlgaeMech() {
    return new AlgaeSubsystem();
  }

  private CoralSubsystem buildCoralMech() {
    return new CoralSubsystem();
  }

  private WristSubsystem buildWrist() {
    return new WristSubsystem();
  }

  private ClimberSubsystem buildClimberSubsystem() {
    return new ClimberSubsystem();
  }

  private RampSubsystem buildRampSubsystem() {
    return new RampSubsystem();
  }

  private VisionSubsystem buildVisionSubsystem() {
    return new VisionSubsystem(
        drivetrain::addVisionMeasurement,
        logger::getHeading,

        // Limelight NetworkTable names
        VisionConstants.camera0Name,
        VisionConstants.camera1Name);
  }

  private USBCameraSubsystem buildUSBCams() {
    return new USBCameraSubsystem();
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public ElevatorSubsystem getElevator() {
    return elevator;
  }

  public AlgaeSubsystem getAlgaeMech() {
    return algae;
  }

  public CoralSubsystem getCoral() {
    return coral;
  }

  public WristSubsystem getWrist() {
    return wrist;
  }

  public ClimberSubsystem getClimber() {
    return climber;
  }

  public RampSubsystem getRamp() {
    return ramp;
  }

  public CommandXboxController getJoystick() {
    return joystick;
  }

  public VisionSubsystem getVision() {
    return vision;
  }

  public USBCameraSubsystem getUSBCams() {
    return usbCams;
  }

  private final AlgaeSubsystem algae;
  private final CoralSubsystem coral;
  private final WristSubsystem wrist;
  private final ElevatorSubsystem elevator;
  private final ClimberSubsystem climber;
  private final RampSubsystem ramp;
  private final VisionSubsystem vision;
  private final USBCameraSubsystem usbCams;

  // Map buttons to trigger variables
  private final JoystickButton redL4 = new JoystickButton(buttonBoard, 1);
  private final JoystickButton redL3 = new JoystickButton(buttonBoard, 2);
  private final JoystickButton redL2 = new JoystickButton(buttonBoard, 3);
  private final JoystickButton redL1 = new JoystickButton(buttonBoard, 4);

  private final JoystickButton blue4 = new JoystickButton(buttonBoard, 5);
  private final JoystickButton blue3 = new JoystickButton(buttonBoard, 6);
  private final JoystickButton blue2 = new JoystickButton(buttonBoard, 7);
  private final JoystickButton blue1 = new JoystickButton(buttonBoard, 8);

  public RobotContainer() {

    // Subsystem initialization
    algae = buildAlgaeMech();
    coral = buildCoralMech();
    wrist = buildWrist();
    elevator = buildElevatorSubsystem();
    climber = buildClimberSubsystem();
    ramp = buildRampSubsystem();
    vision = buildVisionSubsystem();
    usbCams = buildUSBCams();

    // Pathplanner Named Commands (MUST BE DECLARED HERE AND HAVE THE SAME NAME)
    NamedCommands.registerCommand("netAlgae", new NetAlgae(this).withTimeout(1));
    NamedCommands.registerCommand("wrist60Deg", wrist.WristPose(-60).withTimeout(0.3));
    NamedCommands.registerCommand("shootAlgae", algae.outtakeCommand());
    NamedCommands.registerCommand("liftLowAlgae", new PickupReefAlgaeState(this, ALGAE_LOW).withTimeout(1));
    NamedCommands.registerCommand("algaeNeutral", new NeutralAlgae(this).withTimeout(1));
    NamedCommands.registerCommand("liftL4", new ScoreCoralState(this, L4).withTimeout(0.5));
    NamedCommands.registerCommand("liftL3", new ScoreCoralState(this, L3).withTimeout(0.3));
    NamedCommands.registerCommand("scoreCoral", coral.shootCoral().withTimeout(0.3));
    NamedCommands.registerCommand("liftHighAlgae", new PickupReefAlgaeState(this, ALGAE_HIGH).withTimeout(1));
    NamedCommands.registerCommand("liftNet", new NetAlgae(this));
    NamedCommands.registerCommand("scoreNet", algae.outtakeCommand().withTimeout(0.3));
    NamedCommands.registerCommand("neutral", new NeutralState(this).withTimeout(0.5));
    NamedCommands.registerCommand("pickupLolipop",
        new PickupReefAlgae(elevator, wrist, algae, GROUND_ALGAE, 0).withTimeout(1));
    NamedCommands.registerCommand("intakeCoral", coral.intakeCoral());

    autoChooser = AutoBuilder.buildAutoChooser("line");

    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putData("Field", m_field);

    usbCams.setCamera(1);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * governor) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * governor) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate * governor) // Drive counterclockwise with
                                                                                   // negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
    // -joystick.getLeftX()))
    // ));

    // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(0.5).withVelocityY(0))
    // );
    // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    // );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // redL4.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // redL3.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // redL2.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // redL1.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    // joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    /* MICHAEL WAY OF CORAL */
    // joystick.rightBumper().and(joystick.povRight()).onTrue(new ScoreCoralState(this, L4))
    //     .onFalse(new NeutralState(this));
    // joystick.rightTrigger().and(joystick.povRight()).onTrue(new ScoreCoralState(this, L3))
    //     .onFalse(new NeutralState(this));
    // joystick.leftBumper().and(joystick.povRight()).onTrue(new ScoreCoralState(this, L2))
    //     .onFalse(new NeutralState(this));
    // joystick.leftTrigger().and(joystick.povRight()).onTrue(new ScoreCoralState(this, L1))
    //     .onFalse(new NeutralState(this));

    /* Michael Way of Algae */
    // joystick.rightBumper().and(joystick.povLeft()).onTrue(new NetAlgae(this)).onFalse(new NeutralState(this));
    // joystick.rightTrigger().and(joystick.povLeft()).onTrue(new PickupReefAlgaeState(this, ALGAE_HIGH)
    //     .until(() -> algae.holdingAlgae()).finallyDo(() -> new NeutralAlgae(this))).onFalse(new NeutralAlgae(this));
    // joystick.leftBumper().and(joystick.povLeft()).onTrue(new PickupReefAlgaeState(this, ALGAE_LOW)
    //     .until(() -> algae.holdingAlgae()).finallyDo(() -> new NeutralAlgae(this))).onFalse(new NeutralAlgae(this));
    // joystick.leftTrigger().and(joystick.povLeft()).onTrue(new PickupReefAlgae(elevator, wrist, algae, GROUND_ALGAE, 30)
    //     .until(() -> algae.holdingAlgae()).finallyDo(() -> new NeutralAlgae(this))).onFalse(new NeutralAlgae(this));

    // // Left
    // blue4.whileTrue(new DriveRelativeTag(getDrivetrain(),
    //     getVision(),
    //     logger,
    //     new Translation2d(.25, 0),
    //     0));

    // // Right
    // redL4.whileTrue(new DriveRelativeTag(getDrivetrain(),
    //     getVision(),
    //     logger,
    //     new Translation2d(.25, 0),
    //     0));

    /* RUSH WAY OF CORAL */
    redL4.onTrue(
    new ScoreCoralState(this,L4)).onFalse(new NeutralState(this)
    );
    redL3.onTrue(
    new ScoreCoralState(this,L3)).onFalse(new NeutralState(this)
    );
    redL2.onTrue(
    new ScoreCoralState(this,L2)).onFalse(new NeutralState(this)
    );
    redL1.onTrue(
    new ScoreCoralState(this,L1)).onFalse(new NeutralState(this)
    );



    /* RUSH WAY OF ALGAE */
    blue4.onTrue(
    new NetAlgae(this)).onFalse(new NeutralState(this));

    blue3.onTrue(
    new PickupReefAlgaeState(this,ALGAE_HIGH)).onFalse(new NeutralAlgae(this));

    blue2.onTrue(
    new PickupReefAlgaeState(this,ALGAE_LOW)).onFalse(new NeutralAlgae(this));

    blue1.and(()->!algae.holdingAlgae()).onTrue(
      new PickupReefAlgae(elevator,wrist,algae,GROUND_ALGAE,30)).onFalse(new NeutralAlgae(this));

    blue1.and(()->algae.holdingAlgae()).onTrue(
      wrist.WristPose(0))
      .onFalse(new NeutralAlgae(this));

    //joystick.rightBumper().onTrue(new IntakeCoral(this)); // RUSH WAY OF INTAKE
    joystick.a().whileTrue(new ParallelCommandGroup(coral.shootCoral(), algae.outtakeCommand())); // RUSH WAY OF OUTTAKE

    joystick.x().onTrue(new ParallelCommandGroup(new InstantCommand(()->usbCams.setCamera(1)),ramp.lowerRamp().withTimeout(0.25),new IntakeCoral(this))); // Michael WAY OF INTAKE
    // joystick.a().whileTrue(new ParallelCommandGroup(coral.shootCoral(), algae.outtakeCommand())); // Michael WAY OF OUTTAKE
    joystick.b().onTrue(new TOFDrive(drivetrain, 1  ,0.17 ));
    // joystick.leftTrigger().whileTrue(new
    // driveWithSpeed(drivetrain,joystick,0.2)); // Slow Mode
    // joystick.leftBumper().whileTrue(
    // new ParallelTag(
    // drivetrain,
    // vision,
    // logger,
    // () -> -joystick.getLeftY() * MaxSpeed * governor,
    // () -> -joystick.getLeftX() * MaxSpeed * governor
    // )
    // );

    joystick.leftTrigger().onTrue(new driveWithSpeed(drivetrain,joystick,0.05));

    joystick.rightTrigger().onTrue(new driveWithSpeed(drivetrain,joystick,0.35));

    joystick.back().whileTrue(climber.liftRobot()); // Lift Robot (Winch in)

    joystick.start().whileTrue(new ParallelCommandGroup(ramp.liftRamp(), climber.lowerRobot(), new InstantCommand(()->usbCams.setCamera(2)))); // Ramp
                                                                                                                       // Up
                                                                                                                       // Control

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }
}