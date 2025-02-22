package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants.*;

public class WristToPos extends Command {
    private final WristSubsystem wristSubsystem;
    private double setpointAngle;
    private double allowed_error;


    public WristToPos (WristSubsystem wristSubsystem, double setpointAngle){
        this.wristSubsystem = wristSubsystem;
        this.setpointAngle = setpointAngle;
        this.allowed_error = 5;

        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize(){

        MotionMagicConfigs mm = wristSubsystem.getWristConfig().MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(12))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(60));


        Slot0Configs slot0 = wristSubsystem.getWristConfig().Slot0;
        slot0.kS = 0.235; // volts
        slot0.kV = 1; // volts * seconds / distance
        slot0.kA = 0.01; // volts * seconds^2 / distance
        slot0.kP = 12;  // proportional
        slot0.kI = 0;    // integral
        slot0.kD = 0.01;  // derivative
        slot0.kG = 0;
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristSubsystem.getWristMotor().getConfigurator().apply(wristSubsystem.getWristConfig());
    }

    @Override
    public void execute() {
        wristSubsystem.getWristMotor().setControl(wristSubsystem.getMMVCont().withPosition(Units.degreesToRotations(setpointAngle)));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (Units.rotationsToDegrees(wristSubsystem.getWristMotor().getPosition().getValueAsDouble()) < setpointAngle+allowed_error && 
            Units.rotationsToDegrees(wristSubsystem.getWristMotor().getPosition().getValueAsDouble()) > setpointAngle-allowed_error ){
                return true;
            }
        return false;
    }

}