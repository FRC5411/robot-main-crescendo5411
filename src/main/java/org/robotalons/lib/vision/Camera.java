// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.Closeable;
import java.io.IOException;

// -----------------------------------------------------------------[Camera]----------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the odometry and kinematics of a given robot.
 * 
 */
public abstract class Camera implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final CameraStatusContainer STATUS = new CameraStatusContainer();
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  /**
   * Updates the underlying signals within this module.
   */
  public abstract void update();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  public abstract void close() throws IOException;

  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */
  public Boolean getConnected() {
    return STATUS.Connected;
  }
}
