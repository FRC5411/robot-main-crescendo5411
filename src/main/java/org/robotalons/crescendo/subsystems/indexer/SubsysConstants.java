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
public final class SubsysConstants {
  public class DRIVEBASE {
    //MEASUREMENTS
    

    //PORTS


    //OBJECTS


    //DEVICES

  }
  
  public class INDEXER {
    //MEASUREMENTS - placeholder values
    public static final Double TO_CANNON_MOTOR_SPEED = (1d);
    public static final Integer K_CURRENT_LIMIT = (60);

    //PORTS
    public static final Integer TO_CANNON_MOTOR_ID = (22);
    public static final Integer INTAKE_SENSOR_INPUT_ID = (0); //placeholder value
    public static final Integer CANNON_SENSOR_INPUT_ID = (8);
    
    //OBJECTS
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();

    //DEVICES - placeholder values
    public static final Integer K_TO_CANNON_MOTOR_ENCODER_CHANNELA = (0);
    public static final Integer K_TO_CANNON_MOTOR_ENCODER_CHANNELB = (1);
  }
  
  public class INTAKE {
    //MEASUREMENTS
    public static final Double MOTOR_SPEED = (0.45);
    public static final Double OTB_MOTOR_ONE_SPEED = (0.0);
    public static final Double OTB_MOTOR_TWO_SPEED = (0.0);
    public static final Integer K_CURRENT_LIMIT = (25);

    //PORTS
    public static final Integer MOTOR_ID = (0);

    //OBJECTS


    //DEVICES

  }

  public class CANNON {
    //MEASUREMENTS
    

    //PORTS


    //OBJECTS


    //DEVICES

  }
  
  public class ARM {
    //MEASUREMENTS
    

    //PORTS


    //OBJECTS


    //DEVICES

  }
  
  public class VISION {
    //MEASUREMENTS
    

    //PORTS


    //OBJECTS


    //DEVICES

  }
}
