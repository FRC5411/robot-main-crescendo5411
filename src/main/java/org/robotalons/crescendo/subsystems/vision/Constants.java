// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import org.robotalons.lib.vision.Camera;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
    public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29));        
    public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29));
    public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0)); 
    public static final Double SPEAKER_TO_GYRO = 0.1129;
    
    public static final Transform3d SOURCE_CAMERA_POSE = new Transform3d(
      new Translation3d(ROBOT_RADIUS_METERS, new Rotation3d()), new Rotation3d((0d), (0.6806784d), (0d)));
    
    //TODO: Find Real Rotational Values for Speaker
    public static final Transform3d SPEAKER_RIGHT_CAMERA_POSE = new Transform3d(
      new Translation3d(SPEAKER_TO_GYRO, new Rotation3d()), new Rotation3d((0d), (0.6108652d), (0.7853982d)));
    
    public static final Transform3d SPEAKER_LEFT_CAMERA_POSE = new Transform3d(
      new Translation3d(SPEAKER_TO_GYRO, new Rotation3d()), new Rotation3d((0d), (0.6108652d), (-0.7853982d)));
    
    public static final Transform3d INTAKE_CAMERA_POSE = new Transform3d(
      new Translation3d(ROBOT_RADIUS_METERS, new Rotation3d()), new Rotation3d((0d), (-0.08203047d), (1.021018d)));
  }

  public static final class Ports {
    public static final String SOURCE_CAMERA_NAME = ("SOURCE");    
    public static final String FRONT_RIGHT_CAMERA_NAME = ("SPEAKER_RIGHT");    
    public static final String REAR_LEFT_CAMERA_NAME = ("SPEAKER_LEFT");    
    public static final String REAR_RIGHT_CAMERA_NAME = ("INTAKE");    
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {
    public static final Camera SOURCE_CAMERA = (null);
    public static final Camera SPEAKER_FRONT_CAMERA = (null);
    public static final Camera SPEAKER_REAR_CAMERA = (null);
    public static final Camera INTAKE_CAMERA = (null);
  }
}