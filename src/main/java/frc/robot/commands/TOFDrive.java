package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.WristSubsystem;
import static frc.robot.Constants.*;

public class TOFDrive extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CANrange TOFDriveSensor;
    private double distance; // 0.17 for reef
    private double speed;

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public TOFDrive(CommandSwerveDrivetrain drivetrain, double speed, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.speed = speed;

        this.TOFDriveSensor = new CANrange(DRIVETOF_ID);

        addRequirements(drivetrain);
    }

    

    @Override
    public void execute() {
        drivetrain.setControl(robotCentricDrive.withVelocityX(speed).withVelocityY(0));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(robotCentricDrive.withVelocityX(0).withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        if (TOFDriveSensor.getDistance().getValueAsDouble() < distance){ 
            return true;
        }
        return false;
    }

}