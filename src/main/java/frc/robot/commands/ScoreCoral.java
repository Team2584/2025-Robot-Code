package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCoral extends SequentialCommandGroup {

  public ScoreCoral(ElevatorSubsystem elevator, WristSubsystem wrist, CoralSubsystem coral, double coralLevel) {

    addRequirements(elevator, wrist, coral);

    addCommands(

        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.setHeight(coralLevel))).withTimeout(1),
        wrist.WristPose(-68)

    );

  }

}