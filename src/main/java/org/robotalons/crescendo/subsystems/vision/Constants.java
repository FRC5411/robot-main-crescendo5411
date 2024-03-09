// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.photonvision.PhotonCamera;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
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
    
    public static final Transform3d FRONT_LEFT_CAMERA_POSE = new Transform3d(
      new Translation3d((0d), new Rotation3d()), new Rotation3d((0d), (0.6806784d), (0d)));

    public static final Transform3d FRONT_RIGHT_CAMERA_POSE = new Transform3d(
      new Translation3d((0d), new Rotation3d()), new Rotation3d((0d), (0.6108652d), (0.7853982d)));
    
    public static final Transform3d REAR_LEFT_CAMERA_POSE = new Transform3d(
      new Translation3d((0d), new Rotation3d()), new Rotation3d((0d), (0.6108652d), (-0.7853982d)));
    
    public static final Transform3d REAR_RIGHT_CAMERA_POSE = new Transform3d(
      new Translation3d((0d), new Rotation3d()), new Rotation3d((0d), (-0.08203047d), (1.021018d)));
  }

  public static final class Ports {
    public static final String FRONT_LEFT_CAMERA_NAME = ("Arducam_OV9782_USB_Camera");    
    public static final String FRONT_RIGHT_CAMERA_NAME = ("Arducam_OV2311_USB_Camera");    
    public static final String REAR_LEFT_CAMERA_NAME = ("SOURCE"); //Extra Camera
    public static final String REAR_RIGHT_CAMERA_NAME = ("Microsoft_LifeCam_HD-3000");    
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {
    public static final Camera FRONT_LEFT_CAMERA =
      new VisionCamera(
        new PhotonCamera(Ports.FRONT_LEFT_CAMERA_NAME), 
        Constants.Measurements.FRONT_LEFT_CAMERA_POSE);
    public static final Camera FRONT_RIGHT_CAMERA = 
      new VisionCamera(
          new PhotonCamera(Ports.FRONT_RIGHT_CAMERA_NAME), 
          Constants.Measurements.FRONT_RIGHT_CAMERA_POSE);
    public static final Camera REAR_LEFT_CAMERA =       
      new VisionCamera(
          new PhotonCamera(Ports.REAR_LEFT_CAMERA_NAME), 
          Constants.Measurements.REAR_LEFT_CAMERA_POSE);
    public static final Camera REAR_RIGHT_CAMERA =
      new VisionCamera(
        new PhotonCamera(Ports.REAR_RIGHT_CAMERA_NAME), 
        Constants.Measurements.REAR_RIGHT_CAMERA_POSE);
  }
}