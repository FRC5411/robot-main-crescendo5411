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
public final class Constants {
  public class DRIVEBASE {
    //MEASUREMENTS
    

    //PORTS


    //OBJECTS


    //DEVICES

  }
  
  public class INDEXER {
    //MEASUREMENTS
    public static final Double INTAKE_SPARK_MAX_SPEED = (1d);
    public static final Double CANNON_SPARK_MAX_SPEED = (1d);
    public static final Integer K_CURRENT_LIMIT = (60);

    //PORTS
    public static final Integer INTAKE_SPARK_MAX_ID = (0);
    public static final Integer CANNON_SPARK_MAX_ID = (1);
    public static final Integer INTAKE_BREAKBEAM_INPUT_ID = (2);
    public static final Integer CANNON_BREAKBEAM_INPUT_ID = (3);
    
    //OBJECTS
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();

    //DEVICES
    public static final Integer K_INTAKE_SPARK_MAX_ENCODER_CHANNELA = (0);
    public static final Integer K_INTAKE_SPARK_MAX_ENCODER_CHANNELB = (1);
    public static final Integer K_CANNON_SPARK_MAX_ENCODER_CHANNELA = (2);
    public static final Integer K_CANNON_SPARK_MAX_ENCODER_CHANNELB = (3);
  }
  
  public class INTAKE {
    //MEASUREMENTS
    public static final Double MOTOR_SPEED = (0.45);
    public static final Double OTB_MOTOR_ONE_SPEED = (0);
    public static final Double OTB_MOTOR_TWO_SPEED = (0);
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
