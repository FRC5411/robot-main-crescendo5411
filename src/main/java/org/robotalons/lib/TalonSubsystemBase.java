// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.lib.utilities.Operator;

import com.jcabi.aspects.Timeable;

import java.util.concurrent.TimeUnit;
import java.io.Closeable;
// ----------------------------------------------------------[Talon Subsystem Base]---------------------------------------------------------//
/**
 *
 *
 * <h1>TalonSubsystemBase</h1>
 *
 * <p>An extension of the traditional SubsystemBase class which allows for implementation of a pilot control based system.<p>
 * 
 * @see SubsystemBase
 * @author Cody Washington
 */
public abstract class TalonSubsystemBase extends SubsystemBase implements Closeable {
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Talon Subsystem Constructor.
   * @param Name Any name for this subsystem, which can be used for telemetry and logging
   */
  protected TalonSubsystemBase(final String Name) {
    super(Name);
  }
  // --------------------------------------------------------------[Methods]----------------------------------------------------------------//
  /**
   * Configures a pilot's hardware to operate this subsystem
   * @param Operator the next operator of this subsystem instance
   */
  public void configure(final Operator Operator) {

  }
  
  /**
   * Closes this instance and all held resources immediately.
   */
  public void close() {

  } 

  @Override
  @Timeable(limit = 20, unit = TimeUnit.MILLISECONDS)
  public abstract void periodic();
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the current operator of this subsystem.
   * @return Subsystem operator
   */
  public Operator getOperator() {
    return null;
  }
}