// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(34, "rio");

    private String currentColor;

    public CANdleSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    // Wrappers
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void turnGreen() {
        currentColor = "green";
    }

    public Command turnGreenCommand() {
        return runOnce(()->turnGreen());
    }
    
    public void turnOrange() {
        currentColor = "orange";
    }

    public Command turnOrangeCommand() {
        return runOnce(()->turnOrange());
    }

    public void turnOff() {
        currentColor = null;
    }

    public Command turnOffCommand() {
        return runOnce(()->turnOff());
    }

    @Override
    public void periodic() {
        if (currentColor == "green") {
            m_candle.setLEDs(0, 255, 0);
        } else if (currentColor == "orange") {
            m_candle.setLEDs(255, 165, 0);
        } else {
            m_candle.setLEDs(0, 0, 0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Not sure what to put here, fix as needed :) - Shuntao
    }
}
