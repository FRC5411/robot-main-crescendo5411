// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.indexer;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.Lock;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all indexer-wide constants, does not contain robot-wide constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {

  }

  public static final class Ports {

  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
