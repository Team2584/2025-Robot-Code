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

import static frc.robot.Constants.*;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;



public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX climber;
  private final TalonFXConfiguration climberConfigs = new TalonFXConfiguration();



  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public ClimberSubsystem() {
    climber = new TalonFX(ClimberConstants.leftClimberCanId);

    climberConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    climberConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climber.getConfigurator().apply(climberConfigs);
  }

  /**
  * Sets the climb to lift the robot at a contant speed
  */
  public Command liftRobot() {
    return runEnd(()->setVoltage(6), ()->setVoltage(0));
  }

  /**
  * Sets the climb to lower the robot at a contant speed
  */
  public Command lowerRobot() {
    return runEnd(()->setVoltage(-6), ()->setVoltage(0));
  }

  /**
  * Sets the climb Motor to run at a constant specified voltage
  */
  public void setVoltage(double Volts) {
    climber.setControl(voltageRequest.withOutput(Volts));
  }

  
}
