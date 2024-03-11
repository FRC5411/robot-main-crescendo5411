// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
public class IntakeIOSparkMax implements IntakeIO {

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);

  private double appliedVolts = 0.0;

  public IntakeIOSparkMax() {
    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeCurrentLimit);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltsApplied = appliedVolts;
    inputs.currentApplied = new double[] {intakeMotor.getOutputCurrent()};
    inputs.temp = new double[] {intakeMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    intakeMotor.setVoltage(appliedVolts);
  }
}