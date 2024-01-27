// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.robotalons.lib.roller.Roller;
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

  }

  public static final class Ports {
    public static final Integer LAUNCH_CONTROLLER_ID = (0);
    public static final Integer DIRECTIONAL_CONTROLLER_ID = (0);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

    public static final Roller DIRECTIONAL_CONTROLLER = 
      new REVRollerModule(null);
    public static final Roller LAUNCH_CONTROLLER = 
      new REVRollerModule(null);

  }
  
}