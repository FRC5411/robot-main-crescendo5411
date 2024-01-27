// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import org.robotalons.crescendo.subsystems.cannon.REVRollerModule.RollerConstants;
import org.robotalons.lib.roller.Roller;

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
    public static final class Rollers{
      public static final class Directional{
        public static final RollerConstants CONSTANTS = new RollerConstants();
      }
      public static final class Launch{
        public static final RollerConstants CONSTANTS = new RollerConstants();
        static{
          CONSTANTS.GEAR_RATIO = (5d);
        }
      }
    }

    public static final Double ODOMETRY_FREQUENCY = (250d);

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
      new REVRollerModule(Measurements.Rollers.Directional.CONSTANTS);
    public static final Roller LAUNCH_CONTROLLER = 
      new REVRollerModule(Measurements.Rollers.Launch.CONSTANTS);

  }
  
}