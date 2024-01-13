// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.Closeable;
import java.util.ArrayList;
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
 */
public abstract class CommonModule implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Double ODOMETRY_FREQUENCY = (250d);
  private final Constants CONSTANTS;  
  private final Nat<Num> NUMBER;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  protected CommonModuleDataAutoLogged LoggedData = new CommonModuleDataAutoLogged();  
  private List<SwerveModulePosition> PositionDeltas = new ArrayList<>();
  private SwerveModuleState Reference = null;
  private Rotation2d Azimuth_Offset = null;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Common Module Constructor.
   * @param Constants Constants to derive measurements from, which contains 
   * @param Number   Relative number of module, i.e. Nat.N1() would be equivalent to Front Left, and Nat.N2() to Front Right
   */
  protected CommonModule(final Constants Constants, final Nat<Num> Number) {
    CONSTANTS = Objects.requireNonNull(Constants);
    NUMBER = Objects.requireNonNull(Number);
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
   * <p>Describes a given {@link CommonModule}'s measured constants that cannot otherwise be derived through its sensors and hardware.
   */
  public static class Constants {
    public Double POSITION_METERS;    
    public Double WHEEL_RADIUS_METERS;
    public Double AZIMUTH_ENCODER_OFFSET;
    
  }    

  /**
   * <p>Describes a given {@link CommonModule}'s current state of control, and how it should operate given the mode.
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
   * Mutates the module controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Reference Module's new Goal or 'set-point' reference
   * @param Rotation  Module's turning speed 'set-point'
   * @return An optimized version of the reference
   */
  public abstract SwerveModuleState set(final SwerveModuleState Reference, final Double Rotation);


  /**
   * Mutates the module controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of Module control
   */
  public abstract void set(final ReferenceType Mode);
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the internal denotation of this module, i.e. Front Left = Nat.N1(), Front Right = Nat.N2()
   * @return Natural number representation of this module
   */
  public Nat<Num> getDenotation() {
    return NUMBER;
  }
  /**
   * Provides the deltas, or captured data points from odometry from the most recent {@link #periodic()} cycle.
   * @return List of measured module positions
   */
  public List<SwerveModulePosition> getPositionDeltas() {
    return PositionDeltas;
  }

  /**
   * Provides the most-recent cycle observed (measured) position of this module
   * @return Measured module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      LoggedData.Linear_Position_Radians * CONSTANTS.WHEEL_RADIUS_METERS,
      (Objects.isNull(Azimuth_Offset))? 
        (new Rotation2d()):
        (LoggedData.Azimuth_Relative_Position.plus(Azimuth_Offset))
      );
  }

  /**
   * Provides the optimized most-recent reference, or 'set-point' for this module's controller
   * @return Reference module state
   */
  public SwerveModuleState getOptimized() {
    return SwerveModuleState.optimize(Reference, LoggedData.Azimuth_Relative_Position);
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
    LoggedData.Linear_Velocity_Radians_Second * CONSTANTS.WHEEL_RADIUS_METERS, 
    (Objects.isNull(Azimuth_Offset))? 
        (new Rotation2d()):
        (LoggedData.Azimuth_Relative_Position.plus(Azimuth_Offset)));
  }
}
