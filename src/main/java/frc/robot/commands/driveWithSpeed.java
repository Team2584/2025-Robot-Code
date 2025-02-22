package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

public class driveWithSpeed extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;

  private double tempGovernor; // Added to slow MaxSpeed to some %. Set to 1 for full speed.
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1 * Math.PI).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public driveWithSpeed(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, double tempGovernor) {
    this.tempGovernor = tempGovernor;
    this.drivetrain = drivetrain;
    this.joystick = joystick;
  }

  @Override
  public void execute() {
    drivetrain.setControl(drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * tempGovernor) // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed * tempGovernor) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * tempGovernor)); // Drive counterclockwise with negative X (left)
  }

}