// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.util.Units;

import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
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
  // -------------------------------------------------------------[Internal]---------------------------------------------------------------//
  public static final class Measurements {
    public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29));        
    public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29));
    public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0));      
    
    public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((15.4d));

    public static final Double ROBOT_TRANSLATION_KP = (1d);
    public static final Double ROBOT_TRANSLATION_KI = (1d);
    public static final Double ROBOT_TRANSLATION_KD = (1d);    

    public static final Double ROBOT_ROTATIONAL_KP = (1d);
    public static final Double ROBOT_ROTATIONAL_KI = (1d);
    public static final Double ROBOT_ROTATIONAL_KD = (1d);
  }
}
