// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  LEDStrip FRONT_SHOOTER_LED;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    //TODO: Figure out why it wont take in PWM port and length 
    FRONT_SHOOTER_LED = new LEDStrip();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}