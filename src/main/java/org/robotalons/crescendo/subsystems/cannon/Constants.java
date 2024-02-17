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
    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final Double PIVOT_GEAR_RATIO = (1d);

    public static final Double PIVOT_P_GAIN = (1d);
    public static final Double PIVOT_I_GAIN = (1d);
    public static final Double PIVOT_D_GAIN = (1d);

    public static final Double FIRING_P_GAIN = (1d);
    public static final Double FIRING_I_GAIN = (1d);
    public static final Double FIRING_D_GAIN = (1d);

    public static final Double CANNON_LENGTH = (1d);

  }
  
  public static final class Ports {
    public static final Integer FIRING_CONTROLLER_RIGHT_ID = (0);
    public static final Integer FIRING_CONTROLLER_LEFT_ID = (1);
    public static final Integer PIVOT_CONTROLLER_ID = (2);
    public static final Integer PIVOT_ABSOLUTE_ENCODER_ID = (3);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}