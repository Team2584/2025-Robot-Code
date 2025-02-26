package frc.robot.commands;

import java.awt.Robot;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class NeutralAlgae extends ParallelCommandGroup {
  private ElevatorSubsystem elevator;
  private WristSubsystem wrist;
  private AlgaeSubsystem algae;

  public NeutralAlgae(RobotContainer robotContainer) {
    this.elevator = robotContainer.getElevator();
    this.wrist = robotContainer.getWrist();
    this.algae = robotContainer.getAlgaeMech();
    

    addCommands(

        // new InstantCommand(() -> new DriveToPose(drive, () -> new Pose2d())),
        
        algae.setSpeed(-0.05),
        wrist.WristPose(-60).withTimeout(0.7),
        elevator.moveToHeight(0).withTimeout(1).andThen(new InstantCommand(()->elevator.homeElevatorLoop()))
        

    );
    
  }

}