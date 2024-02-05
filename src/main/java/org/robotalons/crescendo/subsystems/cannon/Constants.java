// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all cannon-wide constants, does not contain robot-wide constants.
 *
 * @see CannonSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double EARTH_MASS_KG = (5.972e24);
    public static final Double OBJECT_MASS_KG = (2.35301e-1d);
    public static final Double EARTH_RADIUS_METERS = (6.378e6);
    public static final Double UNIVERSAL_GRAVITATIONAL_CONSTANT = (6.67430e-10d);
  }

  public static final Double ODOMETRY_FREQUENCY = (250d);

  public static final class Ports {
    public static final Integer LAUNCH_LEFT_CONTROLLER_ID = (0);
    public static final Integer LAUNCH_RIGHT_CONTROLLER_ID = (1);
    public static final Integer DIRECTIONAL_CONTROLLER_ID = (2);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}