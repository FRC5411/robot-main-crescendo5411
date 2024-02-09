// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.Closeable;
import java.util.List;
import java.util.Objects;
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
public abstract class Module implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final Constants CONSTANTS;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  protected ModuleStatusContainerAutoLogged Status = new ModuleStatusContainerAutoLogged();  
  protected Rotation2d RotationalAbsoluteOffset = (null);
  protected Rotation2d RotationalRelativeOffset = (null);
  protected SwerveModuleState Reference = (null);
  protected ReferenceType ReferenceMode = ReferenceType.STATE_CONTROL;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Common Module Constructor.
   * @param Constants Constants to derive measurements from, which contains 
   */
  protected Module(final Constants Constants) {
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
    public Double TRANSLATIONAL_GEAR_RATIO = (1d);
    public Double ROTATIONAL_GEAR_RATIO = (1d);
    public Double WHEEL_RADIUS_METERS = (1d);
    public Boolean ROTATIONAL_INVERTED = (false);
    public Boolean TRANSLATIONAL_INVERTED = (false);
    public Double TRANSLATIONAL_MAXIMUM_VELOCITY_METERS = (0d);
    public Double ROTATIONAL_MAXIMUM_VELOCITY_METERS = (0d);
    public Double TRANSLATIONAL_POSITION_METERS = (0d);
    public Rotation2d ROTATIONAL_ENCODER_OFFSET = new Rotation2d();
    public Integer NUMBER = (0);
  }    

  /**
   * <p>Describes a given {@link Module}'s current state of control, and how it should operate given the mode.
   */
  public enum ReferenceType {        
    STATE_CONTROL,
    DISABLED,
    CLOSED,
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the module controller's current 'set-point' or reference state and mutates the module controller's current mode of operation
   * and how it should identify and calculate reference 'set-points'
   * @param Reference Module's new Goal or 'set-point' reference
   * @param Mode Mode of Module control
   * @return An optimized version of the reference
   */
  public final SwerveModuleState set(final SwerveModuleState Reference, final ReferenceType Mode) {
    set(Mode);
    return set(Reference);
  }

  /**
   * Mutates the module controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of Module control
   */
  public void set(final ReferenceType Mode) {
    ReferenceMode = Mode;
  }

  /**
   * Mutates the module controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Reference Module's new Goal or 'set-point' reference
   * @return An optimized version of the reference
   */
  public SwerveModuleState set(final SwerveModuleState Reference) {
    this.Reference = SwerveModuleState.optimize(Reference, getRelativeRotation());
    return this.Reference;
  }

  /**
   * Mutates the module's current applied voltage to the translational controller
   * @param Voltage Next applied voltage
   */
  protected abstract void setTranslationalVoltage(final Double Voltage);

  /**
   * Mutates the module's current applied voltage to the rotational controller
   * @param Voltage next applied voltage
   */
  protected abstract void setRotationalVoltage(final Double Voltage);

  /**
   * Zeroes the rotational relative to offset from the position of the absolute encoders.
   */
  public synchronized void reset() {
    update();
    RotationalAbsoluteOffset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the deltas, or captured data points from odometry from the most recent {@link #periodic()} cycle.
   * @return List of measured module positions
   */
  public abstract List<SwerveModulePosition> getPositionDeltas();

  /**
   * Provides the corresponding timestamps of the deltas from odometry from the most recent {@link #periodic()} cycle.
   * @return List of measured module position timestamps
   */
  public abstract List<Double> getPositionTimestamps();

  /**
   * Provides the internal denotation of this module, i.e. Front Left = 0, Front Right = 1
   * @return Natural number representation of this module
   */
  public Integer getNumber() {
    return CONSTANTS.NUMBER;
  }

  /**
   * Provides the most-recent cycle observed (measured) position of this module
   * @return Measured module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Status.TranslationalPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS,
      Status.RotationalRelativePosition);
  }

  /**
   * Provides the optimized most-recent reference, or 'set-point' for this module's controller
   * @return Reference module state
   */
  public SwerveModuleState getOptimized() {
    return Reference;
  }

  /**
   * Provides the un-optimized most-recent reference, or 'set-point' for this module's controller
   * @return Reference module state
   */
  public SwerveModuleState getReference() {
    return Reference;
  }

  /**
   * Provides the current relative rotation of the module rotational axis
   * @return Rotational axis heading as a relative {@link Rotation2d} object
   */
  public Rotation2d getRelativeRotation() {
    return Status.RotationalRelativePosition.plus(RotationalRelativeOffset);
  }

  /**
   * Provides the current absolute rotation of the module rotational axis
   * @return Rotational axis heading as an absolute {@link Rotation2d} object
   */
  public Rotation2d getAbsoluteRotation() {
    return Status.RotationalAbsolutePosition.plus(RotationalAbsoluteOffset);
  }

  /**
   * Provides the current linear position
   * @return Linear position in meters
   */
  public Double getLinearVelocity() {
    return Status.TranslationalVelocityRadiansSecond * CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
  }

  /**
   * Provides the current linear velocity
   * @return Linear velocity in meters per second
   */
  public Double getLinearPosition() {
    return Status.TranslationalPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS;
  }

  /**
   * Provides the most-recent cycle observed (measured) state of this module
   * @return Measure module state
   */
  public SwerveModuleState getObserved() {
    return new SwerveModuleState(
      Status.TranslationalVelocityRadiansSecond * CONSTANTS.WHEEL_RADIUS_METERS, 
      Status.RotationalRelativePosition);
  }
}
