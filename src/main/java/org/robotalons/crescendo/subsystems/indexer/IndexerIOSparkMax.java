// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;

public class IndexerIOSparkMax implements IndexerIO {

    private CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.indexerID, MotorType.kBrushless);
    private double appliedVolts = 0.0;

  public IndexerIOSparkMax() {
    config();

  }

  public void config(){
    indexerMotor.clearFaults();

    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setSmartCurrentLimit(IndexerConstants.indexerCurrentLimit);
    indexerMotor.enableVoltageCompensation(12.0);

    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerMotor.setInverted(false);

    indexerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.voltageApplied = appliedVolts;
    inputs.currentApplied = indexerMotor.getOutputCurrent();
    inputs.temp = indexerMotor.getMotorTemperature();
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

    indexerMotor.setVoltage(volts);
  }
}