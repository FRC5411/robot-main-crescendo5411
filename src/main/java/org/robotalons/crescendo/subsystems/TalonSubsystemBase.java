// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.lib.utilities.PilotProfile;

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
   * @param Profile the next pilot of this subsystem instance
   */
  public abstract void configure(final PilotProfile Profile);
  
  /**
   * Closes this instance and all held resources immediately.
   */
  public abstract void close();  
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the current pilot of this subsystem.
   * @return Subsystem Pilot
   */
  public abstract PilotProfile getPilot();
}