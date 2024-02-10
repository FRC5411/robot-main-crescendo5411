// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all indexer-wide constants, does not contain robot-wide constants.
 *
 * @see IndexerSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double INDEXER_INTAKE_MOTOR_SPEED = (1d);
    public static final Double INDEXER_CANNON_MOTOR_SPEED = (1d);
    public static final Double INDEXER_INTAKE_MOTOR_DURATION = (1d);
    public static final Double INDEXER_CANNON_MOTOR_DURATION = (1d);
  }

  public static final class Ports {
    public static final Integer INDEXER_INTAKE_MOTOR_ID = (0);
    public static final Integer INDEXER_CANNON_MOTOR_ID = (1);
    public static final Integer INDEXER_INTAKE_RECEIVER_ID = (0);
    public static final Integer INDEXER_CANNON_RECEIVER_ID = (1);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}