// package frc.robot.commandgroup;

// import frc.robot.commands.ArmToPos;
// import frc.robot.commands.IntakeBucket;
// import frc.robot.subsystems.ClawSubsystem;
// import frc.robot.subsystems.FlipperSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;


// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WrapperCommand;

// /** A complex auto command that drives forward, releases a hatch, and then drives backward. */
// public class PickupBucket extends SequentialCommandGroup{

//   public PickupBucket(FlipperSubsystem flipperSubsystem, ClawSubsystem clawSubsystem) {

//     addRequirements(flipperSubsystem, clawSubsystem);

//     addCommands(
        
//         new ParallelCommandGroup(new ArmToPos(flipperSubsystem, -0.47),  new IntakeBucket(clawSubsystem)),

//         new ArmToPos(flipperSubsystem, 0)

//     );
//   }
  
// }