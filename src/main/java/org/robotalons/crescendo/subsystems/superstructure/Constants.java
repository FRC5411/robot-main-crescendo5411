// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.superstructure;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
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
 * @see SuperstructureSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double ABSOLUTE_ENCODER_OFFSET = (0.0075d);

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
    public static final Double PIVOT_MAXIMUM_ROTATION = Units.degreesToRadians((70));

    public static final Double PIVOT_MAXIMUM_RANGE_METERS = (100d);
    public static final Double PIVOT_MINIMUM_RANGE_METERS = (0d);

    public static final InterpolatingMatrixTreeMap<Double,N2,N1> PIVOT_FIRING_MAP = new InterpolatingMatrixTreeMap<>();

    static { //TODO: AUTOMATION TEAM (RECORD SHOT DATA)
      put((197.709382681d), (3000d), Math.PI/2);
    }

    /**
     * Quickly puts the data of a given successful shot relative to the height of the target 
     * @param Magnitude Measured magnitude of the triangle formed by the distance lengthwise and heightwise to a given target 
     * @param Velocity  Measured shooter velocity in RPM units
     * @param Rotation  Measured shooter rotation in radians
     */
    private static void put(final Double Magnitude, final Double Velocity, final Double Rotation) {
      PIVOT_FIRING_MAP.put(Magnitude, MatBuilder.fill(Nat.N2(), Nat.N1(), Velocity, Rotation));
    }
  }
  
  public static final class Ports {
    public static final Integer FIRING_CONTROLLER_RIGHT_ID = (34);
    public static final Integer FIRING_CONTROLLER_LEFT_ID = (35);
    public static final Integer INTAKE_CONTROLLER_ID = (21);
    public static final Integer INDEXER_CONTROLLER_ID = (22);
    public static final Integer PIVOT_CONTROLLER_ID = (32);
    public static final Integer PIVOT_ABSOLUTE_ENCODER_ID = (0);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}