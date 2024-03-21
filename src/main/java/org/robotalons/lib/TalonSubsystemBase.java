// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;
import org.robotalons.lib.utilities.Operator;
import org.robotalons.lib.utilities.TypeVector;

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
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
 * @see TypeVector
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public abstract class TalonSubsystemBase<Keybindings extends Enum<?>, Preferences extends Enum<?>, Operators extends Num> extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<TalonSubsystemBase<?,?,?>> SUBSYSTEMS;
  private final Nat<Operators> ELEMENTS;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Talon Subsystem Constructor.
   * @param Name     Any name for this subsystem, which can be used for telemetry and logging
   * @param Elements Functional interface representing the number of elements required when making calls to {@link #configureOperator(TypeVector)}
   */
  protected TalonSubsystemBase(final String Name, final Nat<Operators> Elements) {
    super(Name);
    ELEMENTS = Elements;
    SUBSYSTEMS.add(this);
    new Alert(Name + " Initialized", AlertType.INFO);
  } static {
    SUBSYSTEMS = new ArrayList<>();
  }
  // --------------------------------------------------------------[Methods]----------------------------------------------------------------//
  /**
   * Configures a subsystem's hardware (GenericHID) to operate this subsystem's Hardware (actuators)
   * @param Operator New subsystem operator
   */
  public synchronized void configureOperator(final TypeVector<Operator<Keybindings, Preferences>, Operators> Operator) {

  }

  /**
   * Configure a subsystem's hardware for PathPlanner autonomous to operate with registered named commands.
   */
  public synchronized void configureAutonomous() {

  }
  
  /**
   * Closes this instance and all held resources (actuators) immediately.
   */
  public synchronized void close() {

  } 

  @Override
  //@Timeable(limit = 20, unit = TimeUnit.MILLISECONDS)
  public synchronized void periodic() {

  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  /**
   * Provides a list (ordered) of all initialized subsystems
   * @return List of subsystems
   */
  public static List<TalonSubsystemBase<?,?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }

  /**
   * Provides a number of elements that should be expected when calling {@link #configureOperator(TypeVector)}
   * @return Functional interface representing a number
   */
  public Nat<Operators> getElements() {
    return ELEMENTS;
  }

  /**
   * Provides the current operator of this subsystem.
   * @return Current subsystem operator
   */
  public TypeVector<Operator<Keybindings, Preferences>, Operators> getOperators() {
    return (null);
  }

}