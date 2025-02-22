package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class NeutralState extends SequentialCommandGroup {
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private CoralSubsystem coral;
  private AlgaeSubsystem algae;

  public NeutralState(RobotContainer robotContainer) {
    this.elevator = robotContainer.getElevator();
    this.wrist = robotContainer.getWrist();
    this.coral = robotContainer.getCoral();
    this.algae = robotContainer.getAlgaeMech();

    addCommands(

        // new InstantCommand(() -> new DriveToPose(drive, () -> new Pose2d())),
        
        
        new ParallelCommandGroup(
          algae.setSpeed(0),
          coral.setSpeedCommand(0),
          wrist.WristPose(-80),
          elevator.moveToHeight(0)
          ).withTimeout(1)
        
        






    );
    
  }

}