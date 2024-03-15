// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.superstructure;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
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
    public static final Double ABSOLUTE_ENCODER_OFFSET = (0.656761d);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final Double PIVOT_GEAR_RATIO = (1d);

    public static final Double PIVOT_P_GAIN = (4.7d);
    public static final Double PIVOT_I_GAIN = (0d);
    public static final Double PIVOT_D_GAIN = (0.695d);

    public static final Double FIRING_P_GAIN = (0.02d);
    public static final Double FIRING_I_GAIN = (0d);
    public static final Double FIRING_D_GAIN = (0.001d);

    public static final Double FIRING_STANDARD_VELOCITY = (2000d);

    public static final Double CANNON_LENGTH = (1d);
    public static final Boolean PIVOT_INVERTED = (false);

    public static final Double SUBWOOFER_LINE = Units.degreesToRadians((41.7d));
    public static final Double AMP_SCORE = Units.degreesToRadians((40.8d));
    public static final Double PODIUM_LINE = Units.degreesToRadians((25.7d));
    public static final Double STARTING_LINE = Units.degreesToRadians((38.0d));
    public static final Double WING_LINE = Units.degreesToRadians((15.1d));

    public static final Double SUBWOOFER_RPM = 750d;


    public static final Double PIVOT_MINIMUM_ROTATION = Units.degreesToRotations((6));
    public static final Double PIVOT_MAXIMUM_ROTATION = Units.degreesToRotations((50));

    public static final Double FIRING_IDLE_PERCENT = (.175d);

    public static final Double FIRING_PASSIVE_PERCENTILE = (0.35d);


    public static final Double OFFSET_WALL_METERS = (1.16d);
    public static final Double PIVOT_MAXIMUM_RANGE_METERS = (3.5d) + OFFSET_WALL_METERS;
    public static final Double PIVOT_MINIMUM_RANGE_METERS = (0d) + OFFSET_WALL_METERS;

    public static final Double SPEAKER_HEIGHT_METERS = (0d);

    public static final Double ALLOWABLE_SHOT_PERCENTAGE = (95e-2);

    public static final Matrix<N2,N1> PIVOT_UPPER_BOUND = MatBuilder.fill(Nat.N2(), Nat.N1(), new double[]{FIRING_STANDARD_VELOCITY, (Units.degreesToRadians((29d)))});
    public static final Matrix<N2,N1> PIVOT_LOWER_BOUND = MatBuilder.fill(Nat.N2(), Nat.N1(), new double[]{FIRING_STANDARD_VELOCITY, (Units.degreesToRadians((55d)))});

    public static final InterpolatingMatrixTreeMap<Double,N2,N1> PIVOT_FIRING_MAP = new InterpolatingMatrixTreeMap<>();

    static {
      PIVOT_FIRING_MAP.put(Math.hypot(PIVOT_MAXIMUM_RANGE_METERS, SPEAKER_HEIGHT_METERS), PIVOT_UPPER_BOUND);
      hyput((0.25), FIRING_STANDARD_VELOCITY, (52d));
      hyput((0.5d), FIRING_STANDARD_VELOCITY, (48d));
      hyput((0.75), FIRING_STANDARD_VELOCITY, (45d));
      hyput((1d), FIRING_STANDARD_VELOCITY, (42.5d));
      hyput((1.25), FIRING_STANDARD_VELOCITY, (40d));
      hyput((1.5d), FIRING_STANDARD_VELOCITY, (38.5d));
      hyput((1.75), FIRING_STANDARD_VELOCITY, (37d));
      hyput((2d), FIRING_STANDARD_VELOCITY, (35.6d));
      hyput((2.25), FIRING_STANDARD_VELOCITY, (34.5d));
      hyput((2.5d), FIRING_STANDARD_VELOCITY, (33.5d));
      hyput((2.75), FIRING_STANDARD_VELOCITY, (32d));
      hyput((3d), FIRING_STANDARD_VELOCITY, (31.1d));
      hyput((3.25d), FIRING_STANDARD_VELOCITY, (30.3d));
      PIVOT_FIRING_MAP.put(Math.hypot(PIVOT_MINIMUM_RANGE_METERS, SPEAKER_HEIGHT_METERS), PIVOT_LOWER_BOUND);
    }

    /**
     * Quickly puts the data of a given successful shot relative to the height of the target
     * @param Magnitude Measured magnitude of the triangle formed by the distance lengthwise and heightwise to a given target
     * @param Velocity  Measured shooter velocity in RPM
     * @param Rotation  Measured shooter rotation in radians
     */
    private static void put(final Double Magnitude, final Double Velocity, final Double Rotation) {
      PIVOT_FIRING_MAP.put(Magnitude, MatBuilder.fill(Nat.N2(), Nat.N1(), Velocity, Rotation));
    }

    /**
     * Quickly puts the data of a given successful shot relative to the height of the target
     * @param Distance  Measured distance of the triangle formed by the distance lengthwise to the speaker
     * @param Velocity  Measured shooter velocity in RPM
     * @param Rotation  Measured shooter rotation in radians
     */
    private static void hyput(final Double Distance, final Double Velocity, final Double Rotation) {
      put(Math.hypot(Distance + OFFSET_WALL_METERS, SPEAKER_HEIGHT_METERS), Velocity, Units.degreesToRadians(Rotation));
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