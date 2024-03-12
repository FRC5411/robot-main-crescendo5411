// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private CANSparkMax indexerMotor = new CANSparkMax(IndexerConstants.indexerID, MotorType.kBrushless);


  public Indexer() {
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

  public void set(double speed){
    indexerMotor.set(speed);
  }

  
}