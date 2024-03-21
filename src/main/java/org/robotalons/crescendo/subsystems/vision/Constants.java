// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
 * <p>
 * Contains all robot-wide constants, does not contain subsystem specific
 * constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Transform3d FRONT_LEFT_CAMERA_POSE = new Transform3d(
        new Translation3d(
            Units.inchesToMeters((-19.4684d)),
            Units.inchesToMeters((26.5215d)),
            Units.inchesToMeters((2.8557d))),
        new Rotation3d((0d), (0.6806784d), (0d)));

    public static final Transform3d FRONT_RIGHT_CAMERA_POSE = new Transform3d(
        new Translation3d(
            Units.inchesToMeters((-9.0316d)),
            Units.inchesToMeters((26.5215d)),
            Units.inchesToMeters((2.8557d))),
        new Rotation3d((0d), (0.6806784d), (0d)));

  }

  public static final class Ports {
    public static final String FRONT_LEFT_CAMERA_NAME = ("Arducam_OV9782_USB_Camera");
    public static final String FRONT_RIGHT_CAMERA_NAME = ("Arducam_OV2311_USB_Camera");
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {

    public static final Camera FRONT_LEFT_CAMERA = new VisionCamera(
        new PhotonCamera(Ports.FRONT_LEFT_CAMERA_NAME),
        Constants.Measurements.FRONT_LEFT_CAMERA_POSE);

    public static final Camera FRONT_RIGHT_CAMERA = new VisionCamera(
        new PhotonCamera(Ports.FRONT_RIGHT_CAMERA_NAME),
        Constants.Measurements.FRONT_RIGHT_CAMERA_POSE);

  }
}