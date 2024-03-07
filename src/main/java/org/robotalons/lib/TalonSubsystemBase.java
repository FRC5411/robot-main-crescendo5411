// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.jcabi.aspects.Timeable;

import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;
import org.robotalons.lib.utilities.Operator;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
// ----------------------------------------------------------[Talon Subsystem Base]---------------------------------------------------------//
/**
 *
 *
 * <h1>TalonSubsystemBase</h1>
 *
 * <p>An extension of the traditional SubsystemBase class which allows for better implementation of a operator-subsystem control based system, 
 * and implementation of standard methods like {@link #close()} <p>
 * 
 * @see SubsystemBase
 * @author Cody Washington
 */
public abstract class TalonSubsystemBase<Keybindings extends Enum<?>, Preferences extends Enum<?>> extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<TalonSubsystemBase<?,?>> SUBSYSTEMS;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Talon Subsystem Constructor.
   * @param Name Any name for this subsystem, which can be used for telemetry and logging
   */
  protected TalonSubsystemBase(final String Name) {
    super(Name);
    SUBSYSTEMS.add(this);
    new Alert(Name + " Initialized", AlertType.INFO);
  } static {
    SUBSYSTEMS = new ArrayList<>();
  }
  // --------------------------------------------------------------[Methods]----------------------------------------------------------------//
  /**
   * Configures a pilot's hardware (GenericHID) to operate this subsystem's Hardware (actuators)
   * @param Operator New subsystem operator
   */
  public synchronized void configure(final Operator<Keybindings, Preferences> Operator) {

  }
  
  /**
   * Closes this instance and all held resources (actuators) immediately.
   */
  public synchronized void close() {

  } 

  /**
   * Provides a safe environment for configuring possibly null operations
   * @param Executable Runnable controller configuration
   */
  protected void with(final Runnable Executable) {
    try {
      Executable.run();
    } catch (final NullPointerException Ignored) {
      new Alert((getName().toUpperCase() + " Bindings Improperly Configured"), AlertType.ERROR);
    }
  }

  @Override
  @Timeable(limit = 20, unit = TimeUnit.MILLISECONDS)
  public synchronized void periodic() {

  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  /**
   * Provides a list (ordered) of all initialized subsystems
   * @return List of subsystems
   */
  public static List<TalonSubsystemBase<?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }

  /**
   * Provides the current operator of this subsystem.
   * @return Current subsystem operator
   */
  public Operator<Keybindings, Preferences> getOperator() {
    return (null);
  }

}