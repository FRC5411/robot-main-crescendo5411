// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.elevator;
import java.io.Closeable;
import java.io.IOException;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.elevator.Constants.Measurements;

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
  private static SparkMaxPIDController pidController;
  private static ElevatorSubsystem instance;

  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(Constants.Ports.ELEVATOR_PORT,MotorType.kBrushless);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.clearFaults();
    elevatorMotor.setSmartCurrentLimit(Constants.Measurements.CURRENT_LIMIT);

    encoder = new Encoder(Constants.Ports.Encoder.CHANNEL_A,Constants.Ports.Encoder.CHANNEL_B);

    pidController = elevatorMotor.getPIDController();

    pidController.setP(Measurements.PID.KP);
    pidController.setI(Measurements.PID.KI);
    pidController.setD(Measurements.PID.KD);
    pidController.setFeedbackDevice((MotorFeedbackSensor) encoder);

    feedforward = new ElevatorFeedforward(
      Constants.Measurements.Feedforward.KS,
      Constants.Measurements.Feedforward.KG, 
      Constants.Measurements.Feedforward.KV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/Speed (-1 =< x <= 1 )", elevatorMotor.get());
    Logger.recordOutput("Elevator/Velocity (RD per sec)", encoder.getDistancePerPulse());
    Logger.recordOutput("Elevator/Position (RD)", encoder.getDistance() * (2.0 * Math.PI /4096.0) / Measurements.GEAR_RATIO);
    Logger.recordOutput("Elevator/Volts", elevatorMotor.getBusVoltage());
  }

  public void setVoltage(int volts){
    elevatorMotor.set(MathUtil.clamp(volts,-12, 12));
  }

  public void pidSet(double demand){
    pidController.setReference(
      demand,
      ControlType.kPosition,
      (69),
      feedforward.calculate(encoder.getDistance() * (2.0 * Math.PI /4096.0) / Measurements.GEAR_RATIO, Measurements.BASE_HEIGHT), 
      SparkMaxPIDController.ArbFFUnits.kVoltage);

  }

  public synchronized void close() {
    elevatorMotor.close();
}


}
