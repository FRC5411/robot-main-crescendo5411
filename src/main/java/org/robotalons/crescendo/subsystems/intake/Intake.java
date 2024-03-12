// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);

  public Intake(){
    config();
  }


  public void config(){
    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeCurrentLimit);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();

    intakeMotor.setInverted(false);
  }

  public void set(double speed){
    intakeMotor.set(speed);
  }




public boolean getBannerSensor() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBannerSensor'");
}

}