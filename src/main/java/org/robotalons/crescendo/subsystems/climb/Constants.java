// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.climb;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all climb-wide constants, does not contain subsystem specific constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double INITIAL_PRESET = (0d);
    public static final Double HIGH_PRESET = (0d);
    public static final Double MID_PRESET = (0d);
    public static final Double LOW_PRESET = (0d);

    public static final Double LEFT_ARM_P = (0.001d);
    public static final Double LEFT_ARM_I = (0d);
    public static final Double LEFT_ARM_D = (0d);

    public static final Double LEFT_ARM_KS = (0.001d);
    public static final Double LEFT_ARM_KV = (0d);
    public static final Double LEFT_ARM_KA = (0d);
    public static final Double LEFT_ARM_KG = (0d);

    public static final Double RIGHT_ARM_P = (0.001d);
    public static final Double RIGHT_ARM_I = (0d);
    public static final Double RIGHT_ARM_D = (0d);

    public static final Double RIGHT_ARM_KS = (0.001d);
    public static final Double RIGHT_ARM_KV = (0d);
    public static final Double RIGHT_ARM_KA = (0d);
    public static final Double RIGHT_ARM_KG = (0d);


  }

  public static final class Ports {
    public static final Integer LEFT_ARM_CONTROLLER_ID = 0; 
    public static final Integer RIGHT_ARM_CONTROLLER_ID = 1;
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
