package frc.robot.commandgroup;

import frc.robot.commands.driveWithSpeed;
import frc.robot.commands.ScoreCoral;

import frc.robot.RobotContainer;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class ScoreCoralState extends ParallelCommandGroup{
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private CoralSubsystem coral;
  

  public ScoreCoralState(RobotContainer robotContainer, double coralLevel) {
    this.drivetrain = robotContainer.drivetrain;
    this.joystick = robotContainer.getJoystick();
    this.elevator = robotContainer.getElevator();
    this.wrist = robotContainer.getWrist();
    this.coral = robotContainer.getCoral();

    addCommands(
        
        new driveWithSpeed(drivetrain, joystick, 0.1),

        new ScoreCoral(elevator, wrist, coral, coralLevel)

    );
  }
  
}