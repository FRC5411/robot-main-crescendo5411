// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.sensors;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.Closeable;
// ---------------------------------------------------------------[Gyroscope]---------------------------------------------------------------//
/**s
 *
 *
 * <p>Singular unit which assists in the control of motion throughout the competition field, which provides implementation for sensing heading
 * and direction of a given drivebase.
 * 
 * @author Cody Washington
 * 
 */
public abstract class Gyroscope implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final GyroscopeStatusContainerAutoLogged STATUS = new GyroscopeStatusContainerAutoLogged();
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  /**
   * Updates the underlying signals within this module.
   */
  public abstract void update();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  public abstract void close();

  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */
  public Boolean getConnected() {
    return STATUS.Connected;
  }

  /**
   * Provides the most-recent rotational velocity of yaw
   * @return Double representing rotational velocity as radians per second
   */
  public Double getYawRotationalVelocity() {
    return STATUS.YawVelocityRadiansSecond;
  }

  /**
   * Provides the most-recent measurement yaw from recent {@link #update()} cycle.
   * @return Rotation in radians of yaw position
   */
  public Rotation2d getYawRotation() {
    return STATUS.YawRotation;
  }

  /**
   * Provides the most-recent measurements of gyroscope's yaw rotation during the most
   * @return Deltas of the gyroscopes yaw rotations
   */
  public Rotation2d[] getOdometryYawRotations() {
    return STATUS.PositionDeltas;
  }
    
}
