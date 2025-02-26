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

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class RampSubsystem extends SubsystemBase {

  private final TalonFX ramp;
  private final TalonFXConfiguration rampConfigs = new TalonFXConfiguration();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public RampSubsystem() {
    ramp = new TalonFX(RampConstants.RAMP_ID);

    // FeedbackConfigs fdb = rampConfigs.Feedback;
    // fdb.SensorToMechanismRatio = RampConstants.RAMP_GEAR_RATIO;

    rampConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    rampConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rampConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    ramp.getConfigurator().apply(rampConfigs);
    ramp.setPosition(0);
  }

  /**
   * Sets the climb to lift the robot at a contant speed
   */
  public Command liftRamp() {
    return runEnd(() -> setVoltage(6), () -> setVoltage(0)).until(() -> (ramp.getPosition().getValueAsDouble() > 6.1));
  }

  /**
   * Sets the climb to lower the robot at a contant speed
   */
  public Command lowerRamp() {
    return runEnd(() -> setVoltage(-6), () -> setVoltage(0))
        .until(() -> (ramp.getPosition().getValueAsDouble() < 0.01));
  }

  /**
   * Sets the climb Motor to run at a constant specified voltage
   */
  public void setVoltage(double Volts) {
    ramp.setControl(voltageRequest.withOutput(Volts));
  }

}
