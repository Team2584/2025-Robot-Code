package frc.robot.commandgroup;

import frc.robot.commands.driveWithSpeed;
import frc.robot.commands.PickupReefAlgae;
import frc.robot.commands.ScoreCoral;

import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class PickupReefAlgaeState extends ParallelCommandGroup{
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private AlgaeSubsystem algae;
  private RobotContainer robotContainer;
  

  public PickupReefAlgaeState(RobotContainer robotContainer, double algaeLevel) {
    this.robotContainer = robotContainer;
    this.joystick = robotContainer.getJoystick();
    this.drivetrain = robotContainer.drivetrain;
    this.elevator = this.robotContainer.getElevator();
    this.wrist = this.robotContainer.getWrist();
    this.algae = this.robotContainer.getAlgaeMech();

    addCommands(
        new driveWithSpeed(drivetrain, joystick, 0.15),

        new PickupReefAlgae(elevator, wrist, algae, algaeLevel, 20.0)

    );
    
  }
  
}