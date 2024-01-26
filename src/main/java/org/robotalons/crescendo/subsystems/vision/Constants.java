// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.robotalons.lib.vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static final class Ports {
    public static final String FRONT_LEFT_CAMERA_NAME = ("");    
    public static final String FRONT_RIGHT_CAMERA_NAME = ("");    
    public static final String REAR_LEFT_CAMERA_NAME = ("");    
    public static final String REAR_RIGHT_CAMERA_NAME = ("");    
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {
    public static final Camera FRONT_LEFT_CAMERA = (null);
    public static final Camera FRONT_RIGHT_CAMERA = (null);
    public static final Camera REAR_LEFT_CAMERA = (null);
    public static final Camera REAR_RIGHT_CAMERA = (null);
  }
}
