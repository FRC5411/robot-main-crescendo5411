// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

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
    public static final Double ABSOLUTE_ENCODER_OFFSET = (0d);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final Double PIVOT_GEAR_RATIO = (1d);

    public static final Double PIVOT_P_GAIN = (5d);
    public static final Double PIVOT_I_GAIN = (0d);
    public static final Double PIVOT_D_GAIN = (0.00001d);

    public static final Double FIRING_P_GAIN = (5d);
    public static final Double FIRING_I_GAIN = (0d);
    public static final Double FIRING_D_GAIN = (0.0001d);

    public static final Double CANNON_LENGTH = (1d);

    public static final Boolean PIVOT_INVERTED = (false);

    public static final Double PIVOT_MINIMUM_ROTATION = Units.degreesToRadians((9));
    public static final Double PIVOT_MAXIMUM_ROTATION = Units.degreesToRadians((60));

    public static final Double PIVOT_MAXIMUM_RANGE_METERS = (100d);
    public static final Double PIVOT_MINIMUM_RANGE_METERS = (0d);

    public static final InterpolatingMatrixTreeMap<Double,N2,N1> PIVOT_FIRING_MAP = new InterpolatingMatrixTreeMap<>();

    static {
      //TODO: AUTOMATION TEAM (FIND DATA POINTS)
      PIVOT_FIRING_MAP.put(ABSOLUTE_ENCODER_OFFSET, MatBuilder.fill(Nat.N2(), Nat.N1(), 0d, 0d));
    }
  }
  
  public static final class Ports {
    public static final Integer FIRING_CONTROLLER_RIGHT_ID = (34);
    public static final Integer FIRING_CONTROLLER_LEFT_ID = (35);
    public static final Integer PIVOT_CONTROLLER_ID = (32);
    public static final Integer PIVOT_ABSOLUTE_ENCODER_ID = (9);
    public static final Integer INDEXER_SENSOR_ID = (8);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}