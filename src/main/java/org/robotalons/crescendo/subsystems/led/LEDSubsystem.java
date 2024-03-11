// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.led;

import org.robotalons.crescendo.subsystems.led.Constants.Measurements;
import org.robotalons.crescendo.subsystems.led.Constants.Ports;
import org.robotalons.crescendo.subsystems.led.LEDStrip.LEDIdentifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  LEDStrip FRONT_SHOOTER_LED;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    FRONT_SHOOTER_LED = new LEDStrip(Measurements.SHOOTER_LED_LENGTH, Ports.SHOOTER_LED_PORT);
    FRONT_SHOOTER_LED.setColor(LEDIdentifier.DARK_RED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    FRONT_SHOOTER_LED.setColor(LEDIdentifier.DARK_BLUE);
  }
}