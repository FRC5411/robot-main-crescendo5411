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
 * <p>Contains all subsystems-wide constants, does not contain robot-wide constants.
 *
 * @see IndexerSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double INDEXER_SPARK_MAX_SPEED = (1d);
    public static final Double INDEXER_SPARK_MAX_DURATION = (1d);
    public static final Double INDEXER_SPARK_MAX_DELTA_THETA = (1d); // Will research whether angle or elapsed time is more effective

    public static final Integer K_INDEXER_INTAKE_SPARK_MAX_ENCODER_CHANNELA = (0);
    public static final Integer K_INDEXER_INTAKE_SPARK_MAX_ENCODER_CHANNELB = (1);
    public static final Integer K_INDEXER_CANNON_SPARK_MAX_ENCODER_CHANNELA = (2);
    public static final Integer K_INDEXER_CANNON_SPARK_MAX_ENCODER_CHANNELB = (3);

    public static final Integer K_CURRENT_LIMIT = (60);
  }

  public static final class Ports {
    public static final Integer INDEXER_INTAKE_SPARK_MAX_ID = (0);
    public static final Integer INDEXER_CANNON_SPARK_MAX_ID = (1);
    public static final Integer INDEXER_INTAKE_BREAKBEAM_INPUT_ID = (2);
    public static final Integer INDEXER_CANNON_BREAKBEAM_INPUT_ID = (3);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
