// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double voltsApplied = 0.0;
    public double currentApplied = 0.0;
    public double temp = 0.0;
  }
 
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVolts(double volts) {}
}