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

    public static final Integer K_INDEXER_INTAKE_MOTOR_ENCODER_CHANNELA = (0);
    public static final Integer K_INDEXER_INTAKE_MOTOR_ENCODER_CHANNELB = (1);

    public static final Integer K_INDEXER_CANNON_MOTOR_ENCODER_CHANNELA = (2);
    public static final Integer K_INDEXER_CANNON_MOTOR_ENCODER_CHANNELB = (3);

    public static final Double INDEXER_INTAKE_MOTOR_P = (0.001d);
    public static final Double INDEXER_INTAKE_MOTOR_I = (0d);
    public static final Double INDEXER_INTAKE_MOTOR_D = (0d);

    public static final Double INDEXER_CANNON_MOTOR_P = (0.001d);
    public static final Double INDEXER_CANNON_MOTOR_I = (0d);
    public static final Double INDEXER_CANNON_MOTOR_D = (0d);

    public static final Double INDEXER_INTAKE_MOTOR_KS = (0.001d);
    public static final Double INDEXER_INTAKE_MOTOR_KV = (0d);
    public static final Double INDEXER_INTAKE_MOTOR_KA = (0d);
    public static final Double INDEXER_INTAKE_MOTOR_KG = (0d);
    
    public static final Double INDEXER_CANNON_MOTOR_KS = (0.001d);
    public static final Double INDEXER_CANNON_MOTOR_KV = (0d);
    public static final Double INDEXER_CANNON_MOTOR_KA = (0d);
    public static final Double INDEXER_CANNON_MOTOR_KG = (0d);

    public static final Integer K_CURRENT_LIMIT = (60);

  }

  public static final class Ports {
    public static final Integer INDEXER_INTAKE_MOTOR_ID = (0);
    public static final Integer INDEXER_CANNON_MOTOR_ID = (1);
    public static final Integer INDEXER_INTAKE_RECEIVER_ID = (2);
    public static final Integer INDEXER_CANNON_RECEIVER_ID = (3);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
