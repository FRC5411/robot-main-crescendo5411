// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.cannon;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.lib.roller.Roller;

// ----------------------------------------------------------[REV Controller Module]--------------------------------------------------------//
/**
 *
 *
 * <h1>REVSimModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes a Flywheel Neo sim.</p>
 * 
 * @see Module
 * @see DrivebaseSubsystem
 */
public class REVRollerModule extends Roller{
  /** Creates a new DrivebaseModuleCANSparkMax. */

  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final RollerConstants CONSTANTS;
  private final Lock ODOMETRY_LOCK;
  private final Queue<Double> ROLLER_QUEUE; 
  private ReferenceType ReferenceMode;

  public RelativeEncoder ROLLER_ENCODER;

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  // private double RotationalRelativePosition = 0;
  // private double RotationalAbsolutePosition = 0;
  private DoubleSupplier SpeedMetersPerSecond = () -> 0;

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Simulator Module Constructor
   * @param Constants Constants of new module instance
   */
  public REVRollerModule(final RollerConstants Constants) {
    super(Constants);
    CONSTANTS = Constants;
    ReferenceMode = ReferenceType.STATE_CONTROL;
    CONSTANTS.ROLLER_CONTROLLER.restoreFactoryDefaults();
    ROLLER_ENCODER = CONSTANTS.ROLLER_CONTROLLER.getEncoder();

    try {
      Thread.sleep((1000));
    } catch (final InterruptedException Ignored) {}

    CONSTANTS.ROLLER_CONTROLLER.setCANTimeout((250));

    CONSTANTS.ROLLER_CONTROLLER.setSmartCurrentLimit((40));
    CONSTANTS.ROLLER_CONTROLLER.enableVoltageCompensation((12.0));

    ROLLER_ENCODER.setPosition((0.0));
    ROLLER_ENCODER.setMeasurementPeriod((10));
    ROLLER_ENCODER.setAverageDepth((2));

    CONSTANTS.ROLLER_CONTROLLER.setIdleMode(IdleMode.kBrake);

    CONSTANTS.ROLLER_CONTROLLER.setCANTimeout((250));

    CONSTANTS.ROLLER_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / org.robotalons.crescendo.subsystems.cannon.Constants.Measurements.ODOMETRY_FREQUENCY));

    ODOMETRY_LOCK = new ReentrantLock();    
    ROLLER_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(ROLLER_ENCODER::getPosition);
    // TODO figure out what time stamps are
    // TIMESTAMP_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.timestamp();
    CONSTANTS.ROLLER_CONTROLLER.burnFlash();
  }

  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class RollerConstants extends Constants {
    public CANSparkMax ROLLER_CONTROLLER;
    public Double GEAR_RATIO;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void close() {
    ROLLER_QUEUE.clear();
  }

  @Override
  public void cease() {
    CONSTANTS.ROLLER_CONTROLLER.set((0d));
  }

  @Override
  public void update() {
    Status.PositionRadians = Units.rotationsToRadians(ROLLER_ENCODER.getPosition()) / CONSTANTS.GEAR_RATIO;
    Status.AppliedVoltage = CONSTANTS.ROLLER_CONTROLLER.getAppliedOutput() * CONSTANTS.ROLLER_CONTROLLER.getBusVoltage();
    Status.VelocityRadiansSecond = Units.rotationsPerMinuteToRadiansPerSecond(ROLLER_ENCODER.getVelocity()) / CONSTANTS.GEAR_RATIO;
  }

  @Override
  public void periodic() {
    ODOMETRY_LOCK.lock();
    update();
    switch(ReferenceMode) {
      case STATE_CONTROL:
          double AdjustReferenceSpeed = SpeedMetersPerSecond.getAsDouble() / CONSTANTS.WHEEL_RADIUS_METERS;
          setVoltage(AdjustReferenceSpeed);
        break;
      case DISABLED:
        cease();
        break;
      case CLOSED:
        close();
        break;
    }
    // TIMESTAMPS.clear();

    ODOMETRY_LOCK.unlock();  
  }

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//


  @Override
  public void set(ReferenceType Mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  // @Override
  // public SwerveModuleState set(final SwerveModuleState Reference) {
  //   this.Reference = SwerveModuleState.optimize(Reference, getAbsoluteRotation());
  //   return this.Reference;
  // }

  public void setVoltage(final Double Demand) {
    CONSTANTS.ROLLER_CONTROLLER.setVoltage(Demand);
  }


  // --------------------------------------------------------------[Acessors]--------------------------------------------------------------//

  @Override
  public Rotation2d getRelativeRotation() {
    return null;
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    return null;
  }

  public Double getLinearPositionRads() {
    return null;
  }
}



