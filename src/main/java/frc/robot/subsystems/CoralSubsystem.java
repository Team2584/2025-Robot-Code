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

import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.CoralMechConstants;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoralMechConstants;
import com.ctre.phoenix6.controls.PositionVoltage;



/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class CoralSubsystem extends SubsystemBase {

  /* Hardware */
  private final TalonFX coral;
  private final CANrange coralSensor;
  private final DigitalInput elevatorCoral;


  /* Configs */
  private static final TalonFXConfiguration coralConfigs = new TalonFXConfiguration();
  private static final CANrangeConfiguration coralSensorConfigs = new CANrangeConfiguration();

  private final PositionVoltage vreq;


  public CoralSubsystem() {
    this.coral = new TalonFX(CoralMechConstants.CORAL_MECH_ID);
    this.coralSensor = new CANrange(CoralMechConstants.CORAL_SENSOR_ID);

    elevatorCoral = new DigitalInput(0);

    vreq = new PositionVoltage(0);


   
    coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

     Slot0Configs slot0 = coralConfigs.Slot0;
        slot0.kS = 0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 8; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output



    coral.getConfigurator().apply(coralConfigs, 0.25);
    coralSensor.getConfigurator().apply(coralSensorConfigs,0.25);

  }

  /**
   * Shoots the coral out. Coral mechanism will continue to spin while being called. Defaults to not moving.
   */
  public Command shootCoral(){
    return runEnd(() -> setSpeed(0.375), () -> setSpeed(0)).withTimeout(2.5);
  }
  

  
  /**
   * Sets the Coral intake to spin forward while being called.
   * Resets speed to zero when holding a Coral or when not being called.
   */
  public Command intakeCoral(){
    return runEnd(() -> setSpeed(0.2), () -> setSpeed(0)).until(()->safeCoral());
  }
  
  /**
  * Sets the speed of the coral rollers 
  */
  public void setSpeed(double speed) {
    coral.set(speed);
  }

  public boolean coralCleared(){
    return !elevatorCoral.get(); // Returns True if obstructed
  }

  public Command setSpeedCommand(double speed) {
    return run(()->setSpeed(speed));
  }

  public boolean hasCoral(){
    if (coralSensor.getDistance().getValueAsDouble() < 0.1){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean safeCoral(){
    if (hasCoral() && !coralCleared()){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Cleared", coralCleared());
        // This method will be called once per scheduler run
    }

  

 
}
