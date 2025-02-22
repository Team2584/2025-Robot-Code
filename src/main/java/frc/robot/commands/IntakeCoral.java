package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.HOME;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCoral extends SequentialCommandGroup {
    private ElevatorSubsystem elevator;
    private WristSubsystem wrist;
    private CoralSubsystem coral;

    public IntakeCoral(RobotContainer robotContainer) {
        this.elevator = robotContainer.getElevator();
        this.wrist = robotContainer.getWrist();
        this.coral = robotContainer.getCoral();



        addCommands(

                new ParallelCommandGroup(
                        elevator.moveToHeight(HOME),
                        coral.intakeCoral(),
                        wrist.WristPose(-70)).until(()->coral.safeCoral())

        );

    }

}