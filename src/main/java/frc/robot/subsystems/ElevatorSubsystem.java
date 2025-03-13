// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableDashboardNumber;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ElevatorSubsystem extends SubsystemBase {

  /* Hardware */
  private final TalonFX leader;
  private final TalonFX follower;
  private final DigitalInput elevatorZeroLimit;

  /* Configs */
  private static final TalonFXConfiguration leaderConfiguration0 = new TalonFXConfiguration();
  private static final TalonFXConfiguration leaderConfiguration1 = new TalonFXConfiguration();
  private Slot0Configs slot0Configs;
  //private Slot1Configs slot1Configs;
  private final MotionMagicConfigs motionMagicConfigs;
  //private final MotionMagicConfigs motionMagicConfigs1;
  private final MotionMagicVoltage motionProfileReq;

  private double setpoint;

  /* Tunable Gains */
  TunableDashboardNumber kS = new TunableDashboardNumber("Elevator/kS", 0.235);
  TunableDashboardNumber kG = new TunableDashboardNumber("Elevator/kG", 0.0);
  TunableDashboardNumber kA = new TunableDashboardNumber("Elevator/kA", 0.01);
  TunableDashboardNumber kV = new TunableDashboardNumber("Elevator/kV", 0.3);
  TunableDashboardNumber kP = new TunableDashboardNumber("Elevator/kP", 12);
  TunableDashboardNumber kI = new TunableDashboardNumber("Elevator/kI", 0.0);
  TunableDashboardNumber kD = new TunableDashboardNumber("Elevator/kD", 0.01);

  TunableDashboardNumber motionCruiseVelocity = new TunableDashboardNumber("Elevator/MotionCruiseVelocity", 0);

  TunableDashboardNumber mm_kA = new TunableDashboardNumber("Elevator/MM_KA", 0.02);
  TunableDashboardNumber mm_kV = new TunableDashboardNumber("Elevator/MM_KV", 0.3);

  /* Status Signals */
  private StatusSignal<Current> supplyLeft;
  private StatusSignal<Current> supplyRight;
  private StatusSignal<Double> closedLoopReferenceSlope;
  double prevClosedLoopReferenceSlope = 0.0;
  double prevReferenceSlopeTimestamp = 0.0;

  public ElevatorSubsystem() {
    /* Instantiate motors and configurators */
    this.leader = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_ID);
    this.follower = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_ID);

    elevatorZeroLimit = new DigitalInput(9);

    FeedbackConfigs fdb = leaderConfiguration0.Feedback;
    fdb.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEAR_RATIO;
    motionProfileReq = new MotionMagicVoltage(0); // This is the motion profile, all setControl must target position
                                                      // based on this (m_)

    this.leader.setPosition(0);

    /* Create Configs */

    // Config 0 

    leaderConfiguration0.CurrentLimits.SupplyCurrentLimit = 60;
    leaderConfiguration0.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfiguration0.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfiguration0.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    slot0Configs = leaderConfiguration0.Slot0; // PID Gains
    
    slot0Configs.kP = 12; // Adjust based on your elevator's needs
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.01;
    slot0Configs.kV = 0.3; // Roughly 1.2V per RPS
    slot0Configs.kA = 0.01; // Roughly 1.2V per RPS
    slot0Configs.kG = 0; // Adds constant value to hold elevator up
    slot0Configs.kS = 0.3; // Static friction compensation
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    motionMagicConfigs = leaderConfiguration0.MotionMagic;


    motionMagicConfigs.MotionMagicCruiseVelocity = 100;
    motionMagicConfigs.MotionMagicAcceleration = 65;
    motionMagicConfigs.MotionMagicExpo_kV = 0.12;

    // Config 1

    leaderConfiguration0.Slot1.kP = 3;//1; // Adjust based on your elevator's needs
    leaderConfiguration0.Slot1.kI = 0;
    leaderConfiguration0.Slot1.kD = 0.1;//0.1;
    leaderConfiguration0.Slot1.kV = 0.12; // Roughly 1.2V per RPS
    leaderConfiguration0.Slot1.kA = 0.01;//10; // Roughly 1.2V per RPS
    leaderConfiguration0.Slot1.kG = 0;//0; // Adds constant value to hold elevator up
    leaderConfiguration0.Slot1.kS = 0;//0.3; // Static friction compensation
    leaderConfiguration0.Slot1.GravityType = GravityTypeValue.Elevator_Static;


    /* Apply Configs */
    leader.getConfigurator().apply(leaderConfiguration0, 0.25);

    // Set up signal monitoring
    supplyLeft = follower.getSupplyCurrent();
    supplyRight = leader.getSupplyCurrent();
    closedLoopReferenceSlope = leader.getClosedLoopReferenceSlope();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, supplyLeft, supplyRight, closedLoopReferenceSlope);

    follower.setControl(new Follower(ElevatorConstants.ELEVATOR_RIGHT_ID, true));

  }

  /**
   * Sets the new target height for the elevator to PID to
   */
  public void setHeight(double heightInches) {
    if (!DriverStation.isEnabled()) {
      return;
    }
    setpoint = heightInches;

    if (setpoint < 3){ //< rotationsToInches(leader.getPosition().getValueAsDouble())) {    
      leaderConfiguration0.MotionMagic.MotionMagicCruiseVelocity = 80;
      leaderConfiguration0.MotionMagic.MotionMagicAcceleration = 30;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(1));
    }
    else if (setpoint > 50){
      leaderConfiguration0.MotionMagic.MotionMagicCruiseVelocity = 200;
      leaderConfiguration0.MotionMagic.MotionMagicAcceleration = 50;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(0));
    }
    else{
      motionMagicConfigs.MotionMagicCruiseVelocity = 200;
      motionMagicConfigs.MotionMagicAcceleration = 75;
      motionMagicConfigs.MotionMagicExpo_kV = 0.12;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(0));
    }

    leader.setControl(motionProfileReq.withPosition(inchesToRotations(heightInches)));
  }

  public Command moveToHeight(double heightInches) {
    setpoint = heightInches;


    if (setpoint < 3){ //< rotationsToInches(leader.getPosition().getValueAsDouble())) {
      leaderConfiguration0.MotionMagic.MotionMagicCruiseVelocity = 80;
      leaderConfiguration0.MotionMagic.MotionMagicAcceleration = 30;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(1));
    }
    else if (setpoint > 50){
      leaderConfiguration0.MotionMagic.MotionMagicCruiseVelocity = 200;
      leaderConfiguration0.MotionMagic.MotionMagicAcceleration = 50;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(0));
    }
    else{
      motionMagicConfigs.MotionMagicCruiseVelocity = 200;
      motionMagicConfigs.MotionMagicAcceleration = 75;
      motionMagicConfigs.MotionMagicExpo_kV = 0.12;
      leader.getConfigurator().apply(leaderConfiguration0);
      leader.setControl(motionProfileReq.withSlot(0));
    }

 
    return runOnce(() -> leader.setControl(motionProfileReq.withPosition(inchesToRotations(heightInches))));
  }

  /**
   * Sets the climb Motors to run at a specified voltage
   */
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  /**
   * Reseeds the motor by setting the current height of the elevator to a
   * specified value.
   */
  public void resetHeight(double newHeightInches) {
    leader.setPosition(inchesToRotations(newHeightInches));
    follower.setPosition(inchesToRotations(newHeightInches));
  }
/** 
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kG.hasChanged(0)
        || kV.hasChanged(0)
        || kA.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
    slot0Configs.kP = 2; // Adjust based on your elevator's needs
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.01;
    slot0Configs.kV = 0.3; // Roughly 1.2V per RPS
    slot0Configs.kA = kA.get(); // Roughly 1.2V per RPS
    slot0Configs.kG = kG.get(); // Adds constant value to hold elevator up
    slot0Configs.kS = 0.3; // Static friction compensation


      motionMagicConfigs.MotionMagicCruiseVelocity = 200;
      motionMagicConfigs.MotionMagicAcceleration = 45;
      motionMagicConfigs.MotionMagicExpo_kV = 0.12;

      //motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();// Rotations per second
      //motionMagicConfigs.MotionMagicExpo_kV = mm_kV.get();
      //motionMagicConfigs.MotionMagicExpo_kA = mm_kA.get();

      leader.getConfigurator().apply(leaderConfiguration0, 0.25);
    }
  }
  **/

  public boolean isZero() {
    return !elevatorZeroLimit.get(); // Returns true if Zero
  }

  public void homeElevator() {
    if (isZero()) {
      resetHeight(0);
    } else {
      return;
    }
  }

  public void homeElevatorLoop() {
    while (!isZero()) {
      continue;
    }
    resetHeight(0);
  }

  public Command homeElevatorCommand() {
    return runOnce(() -> resetHeight(0));
  }

  /**
   * Gets the current height of the elevator in inches
   */
  private double getHeight() {
    return rotationsToInches(leader.getPosition().getValueAsDouble());
  }

  /**
   * Gets the current velocity of the elevator in inches per second
   */
  private double getVelocity() {
    return rotationsToInches(leader.getVelocity().getValueAsDouble());
  }

  /**
   * Converts from inches to rotations of the elevator pulley
   */
  private double inchesToRotations(double heightInches) {
    return (heightInches / (Math.PI * ElevatorConstants.ELEVATOR_PULLEY_PITCH_DIAMETER));
  }

  /**
   * Converts from rotations of the elevator pulley to inches
   */
  private double rotationsToInches(double rotations) {
    return rotations * (Math.PI * ElevatorConstants.ELEVATOR_PULLEY_PITCH_DIAMETER);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", rotationsToInches(leader.getPosition().getValueAsDouble()));
    SmartDashboard.putBoolean("Elevator isZero", isZero());

    // This method will be called once per scheduler run
  }

}
