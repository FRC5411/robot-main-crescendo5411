// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.CANSparkMax;

import org.robotalons.lib.motion.elevator.ElevatorModule;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;


public class REVElevator extends ElevatorModule{

  private final REVElevatorConstants CONSTANTS;
  private ElevatorStates STATE;
  private final Queue<Double> POSITION_QUEUE;
  private final Lock ODOMETRY_LOCK;
  private Rotation2d SETPOINT;
  private DoubleSupplier APPLIED_VOLTS;


  public REVElevator(REVElevatorConstants Constants) {
    super(Constants);
    CONSTANTS = Constants;
    STATE = ElevatorStates.RUNNING;
    APPLIED_VOLTS = () -> 0;
    ODOMETRY_LOCK = new ReentrantLock();
    POSITION_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (STATUS.Position_RD));
  }

  public synchronized void close() {
    POSITION_QUEUE.clear();
    CONSTANTS.elevatorMotor.close();
  }

  @Override
  public void cease() {
    CONSTANTS.elevatorMotor.stopMotor();
  }

  @Override
  public void update() {
    STATUS.Position_RD = CONSTANTS.encoder.get() * CONSTANTS.GEAR_RATIO;
    STATUS.Velocity_MpSec = STATUS.Position_RD * Math.PI / 180 ;
    STATUS.Volts = APPLIED_VOLTS.getAsDouble();
    STATUS.OdometryPosition_RD = 
        POSITION_QUEUE.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.GEAR_RATIO)
            .toArray();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("Elevator/Speed (-1 =< x <= 1 )", elevatorMotor.get());
    // Logger.recordOutput("Elevator/Velocity (RD per sec)", encoder.getDistancePerPulse());
    // Logger.recordOutput("Elevator/Position (RD)", encoder.getDistance() * (2.0 * Math.PI /4096.0) / Measurements.GEAR_RATIO);
    // Logger.recordOutput("Elevator/Volts", elevatorMotor.getBusVoltage());

    ODOMETRY_LOCK.lock();
    switch (STATE) {
      case RUNNING:
        double pidOutput = CONSTANTS.pidController.calculate(
          Math.toDegrees(STATUS.Position_RD),
          SETPOINT.getDegrees()
        );
        double feedForwardOutput = CONSTANTS.feedforward.calculate(
          Math.toRadians(CONSTANTS.pidController.getSetpoint().position),
          CONSTANTS.pidController.getSetpoint().velocity
        );
        setVoltage(pidOutput + feedForwardOutput);
        break;
      case DISABLED:
        cease();
      case CLOSED:
        close();
        break;
    }
    ODOMETRY_LOCK.unlock();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class REVElevatorConstants extends Constants {
    public CANSparkMax elevatorMotor;
    public CANSparkMax rollerMotor;
    public Encoder encoder;
    public ElevatorFeedforward feedforward;
    public ProfiledPIDController pidController;
  }


  // --------------------------------------------------------------[Methods]---------------------------------------------------------------//  
  
    // public void pidSet(double demand){
  //   pidController.setReference(
  //     demand,
  //     ControlType.kPosition,
  //     (69),
  //     feedforward.calculate(encoder.getDistance() * (2.0 * Math.PI /4096.0) / Measurements.GEAR_RATIO, Measurements.BASE_HEIGHT), 
  //     SparkMaxPIDController.ArbFFUnits.kVoltage);

  // }
  
  @Override
  public void setVoltage(double volts) {
    CONSTANTS.elevatorMotor.set(MathUtil.clamp(volts,-12, 12));
  }

  public void setRollerVoltage(double volts){
    CONSTANTS.rollerMotor.set(MathUtil.clamp(volts,-12, 12));
  }

  @Override
  public void setState(ElevatorStates state) {
    STATE = state;
  }


}
