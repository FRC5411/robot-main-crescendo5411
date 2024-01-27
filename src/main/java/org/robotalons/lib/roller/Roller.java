// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.lib.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.Closeable;
import java.util.List;
import java.util.Objects;

import org.robotalons.lib.roller.RollerStatusContainerAutoLogged;
// ----------------------------------------------------------------[Module]---------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the control of motion throughout the competition field, which provides implementation for control of both
 * an azimuth (Rotation) motor, which controls the angle or direction of the module, and a linear (translation) motor which controls the velocity
 * or magnitude of the module.
 * 
 * @see ModuleStatusContainer
 * 
 */
public abstract class Roller extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final Constants CONSTANTS;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  protected RollerStatusContainerAutoLogged Status = new RollerStatusContainerAutoLogged();  
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Common Module Constructor.
   * @param Constants Constants to derive measurements from, which contains 
   */
  protected Roller(final Constants Constants) {
    CONSTANTS = Objects.requireNonNull(Constants);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  /**
   * Immediately closes this object and all held resources, renders it henceforth unusable and
   * no longer queryable through its mutators.
   */
  public abstract void close();

  /**
   * Updates this module to cease all motion and stop actuator outputs immediately regardless of
   * references; but still allows this module to be queried again through its reference mutators.
   */
  public abstract void cease();

  /**
   * Updates inputs properly without performing the rest of the {@linkplain #periodic()} logic necessary
   * due to thread-locking behavior.
   */
  public abstract void update();

  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
   * to be consistent within their own codebases about which responsibilities will be handled by
   * Commands, and which will be handled here.
   */
  public abstract void periodic();
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * <p>Describes a given {@link Module}'s measured constants that cannot otherwise be derived through its sensors and hardware.
   */
  public static class Constants {
    public Double POSITION_METERS = (0d);
    public Double WHEEL_RADIUS_METERS = (1d);
  }    

  /**
   * <p>Describes a given {@link Roller}'s current state of control, and how it should operate given the mode.
   */
  public enum ReferenceType {        
    STATE_CONTROL,
    DISABLED,
    CLOSED,
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the module controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of Module control
   */
  public abstract void set(final ReferenceType Mode);
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  /**
   * Provides the current relative rotation of the module rotational axis
   * @return Rotational axis heading as a relative {@link Rotation2d} object
   */
  public abstract Rotation2d getRelativeRotation();

  /**
   * Provides the current absolute rotation of the module rotational axis
   * @return Rotational axis heading as a absolute {@link Rotation2d} object
   */
  public abstract Rotation2d getAbsoluteRotation();

  /**
   * Provides the internal denotation of this module, i.e. Front Left = 0, Front Right = 1
   * @return Natural number representation of this module
   */

  /**
   * Provides the most-recent cycle observed (measured) position of this module
   * @return Measured module position
   */
}