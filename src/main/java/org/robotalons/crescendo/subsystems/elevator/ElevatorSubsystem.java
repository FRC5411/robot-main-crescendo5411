// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.elevator;
import java.io.Closeable;
import java.io.IOException;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase implements Closeable{
  /** Creates a new ElevatorSubsystem. */
  private static CANSparkMax elevatorMotor;
  private static Encoder encoder;
  private static ElevatorFeedforward feedforward;
  private static ElevatorSubsystem instance;

  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(Constants.Ports.ELEVATOR_PORT,MotorType.kBrushless);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.clearFaults();
    elevatorMotor.setSmartCurrentLimit(Constants.Measurements.CURRENT_LIMIT);

    encoder = new Encoder(Constants.Ports.Encoder.CHANNEL_A,Constants.Ports.Encoder.CHANNEL_B);

    feedforward = new ElevatorFeedforward(
      Constants.Measurements.Feedforward.KS,
      Constants.Measurements.Feedforward.KG, 
      Constants.Measurements.Feedforward.KV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator Speed", elevatorMotor.get());
  }

  public void setVoltage(int volts){
    elevatorMotor.set(MathUtil.clamp(volts,-12, 12));
  }

  public synchronized void close() {
    elevatorMotor.close();
}


}
