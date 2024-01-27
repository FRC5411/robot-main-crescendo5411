// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.cannon;
// unfortunately most of this code is copied from cody since my understanding of tcb is very little

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

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
  private final Queue<Double> LINEAR_QUEUE; 
  private ReferenceType ReferenceMode;

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  // private double RotationalRelativePosition = 0;
  // private double RotationalAbsolutePosition = 0;

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Simulator Module Constructor
   * @param Constants Constants of new module instance
   */
  public REVRollerModule(final RollerConstants Constants) {
    super(Constants);
    CONSTANTS = Constants;
    ReferenceMode = ReferenceType.STATE_CONTROL;

    ODOMETRY_LOCK = new ReentrantLock();

    LINEAR_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (Status.PositionRadians));
  }

  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class RollerConstants extends Constants {
    public CANSparkMax DIRECTIONAL_CONTROLLER;
    public CANSparkMax LAUNCH_CONTROLLER;
    public RelativeEncoder DIRECTIONAL_ENCODER;
    public RelativeEncoder LAUNCH_ENCODER;
    public PIDController DIRECTIONAL_CONTROLLER_PID;
    public PIDController LAUNCH_CONTROLLER_PID;
    public SimpleMotorFeedforward LINEAR_CONTROLLER_FEEDFORWARD;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void close() {
    LINEAR_QUEUE.clear();
  }

  @Override
  public void cease() {
    CONSTANTS.DIRECTIONAL_CONTROLLER.set((0d));
    CONSTANTS.LAUNCH_CONTROLLER.set((0d));
  }

  @Override
  public void update() {

  }

  @Override
  public void periodic() {

  }

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//


  @Override
  public void set(ReferenceType Mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
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



