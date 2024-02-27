// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.elevator;
import java.io.Closeable;
import java.io.IOException;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase implements Closeable{
  /** Creates a new ElevatorSubsystem. */
  private static CANSparkMax elevatorMotor;
  private static ElevatorSubsystem instance;

  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(Constants.Ports.ELEVATOR_PORT,MotorType.kBrushless);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.clearFaults();

    elevatorMotor.setSmartCurrentLimit(Constants.Measurements.CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage(int volts){
    elevatorMotor.set(MathUtil.clamp(volts,-12, 12));
  }

  public synchronized void close() {
    elevatorMotor.close();
}


}
