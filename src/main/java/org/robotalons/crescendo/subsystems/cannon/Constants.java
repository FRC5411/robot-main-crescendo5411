// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import java.util.function.Function;
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
    public static final Double EARTH_MASS_KG = (5.972e24);
    public static final Double EARTH_AIR_DENSITY = (1.1839d);
    public static final Double EARTH_RADIUS_METERS = (6.378e6);

    public static final Double OBJECT_MAXIMUM_LINEAR_VELOCITY= (1d);
    public static final Double OBJECT_CYLINDER_DRAG_MU = (1.17d);
    public static final Double OBJECT_MASS_KG = (2.35301e-1d);
    public static final Double OBJECT_INNER_RADIUS = Units.inchesToMeters((10d));
    public static final Double OBJECT_OUTER_RADIUS = Units.inchesToMeters((14d));
    public static final Double OBJECT_VOLUME = 
      (1/4) * Math.pow(Math.PI,(2)) * Math.pow((OBJECT_OUTER_RADIUS - OBJECT_INNER_RADIUS),(2)) * (OBJECT_OUTER_RADIUS + OBJECT_INNER_RADIUS); 
    public final Function<Rotation2d, Double> OBJECT_HORIZONTAL_CROSS_SECTION = (final Rotation2d Theta) -> 
      (Math.PI * (OBJECT_OUTER_RADIUS - OBJECT_INNER_RADIUS)) * (Math.abs(Math.cos(Theta.getRadians())) + 1) * Math.PI * OBJECT_OUTER_RADIUS;
    public final Function<Rotation2d, Double> OBJECT_VERTICAL_CROSS_SECTION = (final Rotation2d Theta) ->
      (Math.PI * (OBJECT_OUTER_RADIUS - OBJECT_INNER_RADIUS)) * (Math.abs(Math.sin(Theta.getRadians())) + 1) * Math.PI * OBJECT_OUTER_RADIUS; 
    
    public static final Double UNIVERSAL_GRAVITATIONAL_CONSTANT = (6.67430e-10d);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final Double PIVOT_GEAR_RATIO = (1d);

    public static final Double PIVOT_P_GAIN = (1d);
    public static final Double PIVOT_I_GAIN = (1d);
    public static final Double PIVOT_D_GAIN = (1d);

    public static final Double FIRING_P_GAIN = (1d);
    public static final Double FIRING_I_GAIN = (1d);
    public static final Double FIRING_D_GAIN = (1d);

    public static final Double FIRING_LENGTH = (1d);

  }
  
  public static final class Ports {
    public static final Integer FIRING_CONTROLLER_RIGHT_ID = (0);
    public static final Integer FIRING_CONTROLLER_LEFT_ID = (1);
    public static final Integer PIVOT_CONTROLLER_ID = (2);
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}