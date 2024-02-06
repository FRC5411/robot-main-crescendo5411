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
    public static final Double EARTH_RADIUS_METERS = (6.378e6);

    public static final Double OBJECT_MASS_KG = (2.35301e-1d);
    public static final Double OBJECT_INNER_RADIUS = (10d);
    public static final Double OBJECT_OUTER_RADIUS = (14d);
    public static final Double OBJECT_VOLUME = 
      (1/4) * Math.pow(Math.PI,(2)) * Math.pow((OBJECT_OUTER_RADIUS - OBJECT_INNER_RADIUS),(2)) * (OBJECT_OUTER_RADIUS + OBJECT_INNER_RADIUS);    

    
    public static final Double UNIVERSAL_GRAVITATIONAL_CONSTANT = (6.67430e-10d);

    public static final Double ODOMETRY_FREQUENCY = (250d);
  }
  
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