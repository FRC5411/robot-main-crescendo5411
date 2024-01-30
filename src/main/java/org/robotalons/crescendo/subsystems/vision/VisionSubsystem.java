// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.robotalons.lib.vision.Camera;

import java.io.Closeable;
import java.util.List;
// ------------------------------------------------------------[Vision Subsystem]----------------------------------------------------------//
/**
 *
 *
 * <h1>VisionSubsystem</h1>
 *
 * <p>Utility class which controls the operation of cameras to create accurate pose estimation and odometry.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public final class VisionSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final List<Camera> CAMERAS;
  public static final PhotonCamera SOURCE;
  public static final PhotonCamera SPEAKER_FRONT;
  public static final PhotonCamera SPEAKER_REAR;
  public static final PhotonCamera INTAKE;

  public static final VisionCamera SOURCE_CAMERA;
  public static final VisionCamera SPEAKER_FRONT_CAMERA;
  public static final VisionCamera SPEAKER_REAR_CAMERA;
  public static final VisionCamera INTAKE_CAMERA;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static VisionSubsystem Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Subsystem Constructor.
   */
  private VisionSubsystem() {} static {
    SOURCE = new PhotonCamera("Camera_1");
    SPEAKER_FRONT = new PhotonCamera("Camera_2");
    SPEAKER_REAR = new PhotonCamera("Camera_3");
    INTAKE = new PhotonCamera("Camera_4(OD)");

    SOURCE_CAMERA = new VisionCamera(
      SOURCE, 
      Constants.Measurements.SOURCE_CAMERA_POSE, 
      SOURCE.getName()
    );

    SPEAKER_FRONT_CAMERA = new VisionCamera(
      SPEAKER_FRONT, 
      Constants.Measurements.SPEAKER_FRONT_CAMERA_POSE, 
      SPEAKER_FRONT.getName()
    );

    SPEAKER_REAR_CAMERA = new VisionCamera(
      SPEAKER_REAR, 
      Constants.Measurements.SPEAKER_REAR_CAMERA_POSE, 
      SPEAKER_REAR.getName()
    );

    INTAKE_CAMERA = new VisionCamera(
      INTAKE, 
      Constants.Measurements.INTAKE_CAMERA_POSE, 
      INTAKE.getName()
    );

    CAMERAS = List.of(
      SOURCE_CAMERA,
      SPEAKER_FRONT_CAMERA,
      SPEAKER_REAR_CAMERA,
      INTAKE_CAMERA
    ); 
  }
  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  public synchronized void close() {

  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized VisionSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance)) {
        Instance = new VisionSubsystem();
      }
      return Instance;
  }
}
