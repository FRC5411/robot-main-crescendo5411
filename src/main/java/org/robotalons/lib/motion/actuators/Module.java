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
// -------------------------------------------------------------[Common Module]-------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the control of motion throughout the competition field, which provides implementation for control of both
 * an azimuth (Rotation) motor, which controls the angle or direction of the module, and a linear (translation) motor which controls the velocity
 * or magnitude of the module.
 * 
 * @see CommonModuleDataAutoLogged
 * 
 */
public abstract class Module implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final Constants CONSTANTS;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  protected ModuleStatusContainerAutoLogged Status = new ModuleStatusContainerAutoLogged();  
  protected SwerveModuleState Reference = null;
  protected Rotation2d Azimuth_Offset = null;
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
    public Double LINEAR_GEAR_RATIO = (1d);
    public Double ROTATION_GEAR_RATIO = (1d);
    public Double POSITION_METERS = (0d);
    public Double WHEEL_RADIUS_METERS = (1d);
    public Double AZIMUTH_ENCODER_OFFSET = (0d);
    public Integer NUMBER;
    
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
   * Mutates the module controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Reference Module's new Goal or 'set-point' reference
   * @return An optimized version of the reference
   */
  public abstract SwerveModuleState set(final SwerveModuleState Reference);

  /**
   * Mutates the module controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of Module control
   */
  public abstract void set(final ReferenceType Mode);
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the deltas, or captured data points from odometry from the most recent {@link #periodic()} cycle.
   * @return List of measured module positions
   */
  public abstract List<SwerveModulePosition> getPositionDeltas();

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
  public Integer getNumber() {
    return CONSTANTS.NUMBER;
  }

  /**
   * Provides the most-recent cycle observed (measured) position of this module
   * @return Measured module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Status.LinearPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS,
      (Objects.isNull(Azimuth_Offset))? 
        (new Rotation2d()):
        (Status.RotationalRelativePosition.plus(Azimuth_Offset))
      );
  }

  /**
   * Provides the optimized most-recent reference, or 'set-point' for this module's controller
   * @return Reference module state
   */
  public SwerveModuleState getOptimized() {
    return SwerveModuleState.optimize(Reference, Status.RotationalRelativePosition);
  }

  /**
   * Provides the un-optimized most-recent reference, or 'set-point' for this module's controller
   * @return Reference module state
   */
  public SwerveModuleState getReference() {
    return Reference;
  }

  /**
   * Provides the most-recent cycle observed (measured) state of this module
   * @return Measure module state
   */
  public SwerveModuleState getObserved() {
    return new SwerveModuleState(
      Status.LinearVelocityRadiansSecond * CONSTANTS.WHEEL_RADIUS_METERS, 
    (Objects.isNull(Azimuth_Offset))? 
        (new Rotation2d()):
        (Status.RotationalRelativePosition.plus(Azimuth_Offset)));
  }
}
