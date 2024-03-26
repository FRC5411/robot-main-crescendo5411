// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import javax.validation.constraints.NotNull;
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
public abstract class TalonSubsystemBase<@NotNull Keybindings extends Enum<?>, @NotNull Preferences extends Enum<?>, @NotNull Operators extends Num> extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<TalonSubsystemBase<?,?,?>> SUBSYSTEMS;
  private final Nat<Operators> ELEMENTS;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Talon Subsystem Constructor.
   * @param Name     Any name for this subsystem, which can be used for telemetry and logging
   * @param Elements Functional interface representing the number of elements required when making calls to {@link #configure(TypeVector)}
   */
  protected TalonSubsystemBase(final String Name, final Nat<Operators> Elements) {
    super(Objects.requireNonNull(Name));
    ELEMENTS = Objects.requireNonNull(Elements);
    SUBSYSTEMS.add(this);
  } static {
    SUBSYSTEMS = new ArrayList<>();
  }
  // --------------------------------------------------------------[Methods]----------------------------------------------------------------//
  /**
   * Configures a subsystem's hardware (GenericHID) to operate this subsystem's Hardware (actuators)
   * @param Operator New subsystem operator
   */
  public synchronized void configure(final TypeVector<Operator<Keybindings, Preferences>, Operators> Operator) {}
  
  /**
   * Closes this instance and all held resources (actuators) immediately.
   */
  @Override
  public abstract void close() throws IOException;
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  /**
   * Provides a list (ordered) of all initialized subsystems
   * @return List of subsystems
   */
  public static List<TalonSubsystemBase<?,?,?>> getSubsystems() {
    return SUBSYSTEMS;
  }

  /**
   * Provides a number of elements that should be expected when calling {@link #configure(TypeVector)}
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