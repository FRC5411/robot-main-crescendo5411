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

    public static final Double CONTROLLER_ARM_SPEED = (0.4);

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

    public static final Double K_TICKS_2_RAD = (2.0 * Math.PI /4096.0);

    public static final Integer K_CURRENT_LIMIT = (60);

    public static final float K_FORWARD_ARM_LIMIT = (90);
    public static final float K_REVERSE_ARM_LIMIT = (-90);

    //TODO: Fix the channels
    public static final Integer K_RIGHT_FORWARD_CHANNELA = (0);
    public static final Integer K_RIGHT_FORWARD_CHANNELB = (1);

    public static final Integer K_LEFT_FORWARD_CHANNELA = (2);
    public static final Integer K_LEFT_FORWARD_CHANNELB = (3);

    //TODO: Find the gear ratio
    public static final Double K_GEARRATIO = (1/1.0);


  }

  public static final class Ports {
    //TODO: Find out Motor IDS
    public static final Integer LEFT_ARM_CONTROLLER_ID = 0; 
    public static final Integer RIGHT_ARM_CONTROLLER_ID = 1;
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
