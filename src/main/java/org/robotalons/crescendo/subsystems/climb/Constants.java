// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.climb;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    public static final double LEFT_ARM_P = (2d);
    public static final double LEFT_ARM_I = (0d);
    public static final double LEFT_ARM_D = (0.0001d);

    public static final double LEFT_ARM_KS = (0.001d);
    public static final double LEFT_ARM_KV = (0d);
    public static final double LEFT_ARM_KA = (0d);
    public static final double LEFT_ARM_KG = (0d);

    public static final double RIGHT_ARM_P = (2d);
    public static final double RIGHT_ARM_I = (0d);
    public static final double RIGHT_ARM_D = (0.0001d);

    public static final double RIGHT_ARM_KS = (0.001d);
    public static final double RIGHT_ARM_KV = (0d);
    public static final double RIGHT_ARM_KA = (0d);
    public static final double RIGHT_ARM_KG = (0d);

    public static final double LEFT_ENCODER_OFFSET = (0d);
    public static final double RIGHT_ENCODER_OFFSET = (0d);

    public static final double TICK_CONVERSION_FACTOR = (2.0 * Math.PI /4096.0);

    public static final double MINIMUM_ARM_ROTATION = Units.degreesToRadians((-100d));
    public static final double MAXIMUM_ARM_ROTATION = Units.degreesToRadians((100d));

    public static final int CURRENT_LIMIT = (60);

    public static final double GEAR_RATIO = (1/244.5);

    public static final Rotation2d HOLD_PARALLEL_GROUND = Rotation2d.fromDegrees((90));
    public static final Rotation2d HOLD_STRAIGHT = Rotation2d.fromDegrees((45));

  }

  public static final class Ports {
    public static final int LEFT_ARM_CONTROLLER_ID = (41);
    public static final int RIGHT_ARM_CONTROLLER_ID = (42);
    public static final int LEFT_ENCODER_ID = (2);
    public static final int RIGHT_ENCODER_ID = (3);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }

}