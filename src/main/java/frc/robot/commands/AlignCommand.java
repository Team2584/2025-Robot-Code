// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.VisionConstants;

// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.LimelightHelpers;

// public class AlignCommand extends Command {
//   private PIDController xController, yController, rotController;
//   private boolean isRightScore;
//   private VisionSubsystem m_Vision;
//   private Timer dontSeeTagTimer, stopTimer;
//   private CommandSwerveDrivetrain m_Swerve;

//   private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//   private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();


//   public AlignCommand(CommandSwerveDrivetrain swerve, VisionSubsystem limelight, boolean isRightScore) {
//     xController = new PIDController(0.15, 0, 0);  // Vertical movement
//     yController = new PIDController(0.5, 0, 0);  // Horitontal movement
//     rotController = new PIDController(0.03, 0, 0);  // Rotation
//     this.isRightScore = isRightScore;
//     this.m_Vision = limelight;
//     this.m_Swerve = swerve;
//     addRequirements(swerve, limelight);
//   }

//   @Override
//   public void initialize() {
//     this.stopTimer = new Timer();
//     this.stopTimer.start();
//     this.dontSeeTagTimer = new Timer();
//     this.dontSeeTagTimer.start();

//     rotController.setSetpoint(0);
//     rotController.setTolerance(0.5);

//     xController.setSetpoint(-0.5);
//     xController.setTolerance(0.005);

//     yController.setSetpoint(isRightScore ? 0.4 : 0.4);
//     yController.setTolerance(0.0005);
//   }

//   @Override
//   public void execute() {
//     if (m_Vision.getTV()) {
//       this.dontSeeTagTimer.reset();

//       double[] postions = LimelightHelpers.getBotPose_TargetSpace(VisionConstants.camera0Name);
//       SmartDashboard.putNumber("x", postions[2]);
      
//       double xSpeed = xController.calculate(postions[2]);
//       //SmartDashboard.putNumber("xspee", xSpeed);
//       double ySpeed = -yController.calculate(postions[0]);
//       double rotValue = rotController.calculate(postions[4]);
//       m_Swerve.setControl(alignRequest.withVelocityX(yController.getError() < 0.0005 ? xSpeed : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));

//       m_Swerve.setControl(alignRequest.withVelocityX(yController.getError() < 0.0005 ? 0 : 0).withRotationalRate(rotValue).withVelocityY(ySpeed));


//       if (!rotController.atSetpoint() ||
//           !yController.atSetpoint() ){//||
//           //!xController.atSetpoint()) {
//         stopTimer.reset();
//       }
//     } else {
//       m_Swerve.setControl(idleRequest);
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     m_Swerve.setControl(idleRequest);
//   }

//   @Override
//   public boolean isFinished() { 
//     return this.dontSeeTagTimer.hasElapsed(1) ||
//         stopTimer.hasElapsed(0.3);
//   }
// }