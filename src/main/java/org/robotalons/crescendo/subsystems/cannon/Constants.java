// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.util.Units;

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
    public static final Double ABSOLUTE_ENCODER_OFFSET = (0d);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final Double PIVOT_GEAR_RATIO = (1d);

    public static final Double PIVOT_P_GAIN = (5d);
    public static final Double PIVOT_I_GAIN = (0d);
    public static final Double PIVOT_D_GAIN = (0.00001d);

    public static final Double FIRING_P_GAIN = (5d);
    public static final Double FIRING_I_GAIN = (0d);
    public static final Double FIRING_D_GAIN = (0.0001d);

    public static final Double CANNON_LENGTH = (1d);

    public static final Double PIVOT_MINIMUM_ROTATION = Units.degreesToRadians((9));
    public static final Double PIVOT_MAXIMUM_ROTATION = Units.degreesToRadians((60));

  }
  
  public static final class Ports {
    public static final Integer FIRING_CONTROLLER_RIGHT_ID = (34);
    public static final Integer FIRING_CONTROLLER_LEFT_ID = (35);
    public static final Integer PIVOT_CONTROLLER_ID = (32);
    public static final Integer PIVOT_ABSOLUTE_ENCODER_ID = (9);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}