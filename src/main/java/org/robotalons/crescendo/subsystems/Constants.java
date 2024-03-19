// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.util.Units;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all subsystem-wide constants, does not contain subsystem specific constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  //localhost:56384
  // -------------------------------------------------------------[Internal]---------------------------------------------------------------//
  public static final class Measurements {
    public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29.5));        
    public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29.5));
    public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0));      
    
    public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((15.4d));
    public static final Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

    public static final Double ROBOT_MAXIMUM_LINEAR_ACCELERATION = Units.feetToMeters((9.5d));
    public static final Double ROBOT_MAXIMUM_ANGULAR_ACCELERATION = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

    public static final Double ROBOT_TRANSLATION_KP = (1d);
    public static final Double ROBOT_TRANSLATION_KI = (0d);
    public static final Double ROBOT_TRANSLATION_KD = (0d);    

    public static final Double ROBOT_ROTATIONAL_KP = (1d);
    public static final Double ROBOT_ROTATIONAL_KI = (0d);
    public static final Double ROBOT_ROTATIONAL_KD = (0d);
  }
}