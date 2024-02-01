// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.robotalons.lib.vision.Camera;

import java.io.Closeable;
import java.io.IOException;
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
   * Retrieves the existing instance of this static utility class.
   * @return Utility class's instance
   */
  public static synchronized VisionSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance)) {
        Instance = new VisionSubsystem();
      }
      return Instance;
  }

  /**
   * Retreives the Pose3d of the robot that is averaged from the two camera estimations.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID
   * @return New approximation of Pose3d from robot. 
   */
  public static Pose3d getApproximatedRobotPose(Integer CAMERA1_ID, Integer CAMERA2_ID){
    Camera CAMERA1 = CAMERAS.get(CAMERA1_ID - 1);
    Camera CAMERA2 = CAMERAS.get(CAMERA2_ID - 1);

    Pose3d CAMERA1_POSE = CAMERA1.getRobotPosition();
    Pose3d CAMERA2_POSE = CAMERA2.getRobotPosition();

    double avgX = (CAMERA1_POSE.getX() + CAMERA2_POSE.getX()) / 2;
    double avgY = (CAMERA1_POSE.getY() + CAMERA2_POSE.getY()) / 2;
    double avgZ = (CAMERA1_POSE.getZ() + CAMERA2_POSE.getZ()) / 2;

    double avgPitch = (CAMERA1_POSE.getRotation().getX() + CAMERA2_POSE.getRotation().getX()) / 2;
    double avgRoll = (CAMERA1_POSE.getRotation().getY() + CAMERA2_POSE.getRotation().getY()) / 2;
    double avgYaw = (CAMERA1_POSE.getRotation().getZ() + CAMERA2_POSE.getRotation().getZ()) / 2;


    return new Pose3d(avgX, avgY, avgZ, new Rotation3d(avgPitch, avgRoll, avgYaw));
  }

  /**
   * Closes Vision Subystem.
   */
  public static void closeCameras() throws IOException{
    SOURCE_CAMERA.close();
    SPEAKER_FRONT_CAMERA.close();
    SPEAKER_REAR_CAMERA.close();
    INTAKE_CAMERA.close();
  }

  /**
   * Takes snapshot from specified Camera integer.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   */
  public void snapshot(Integer CAMERA_ID){
    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    CAMERA.snapshot();
  }

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query of specific camera called.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @return List of robot relative snapshot time deltas of specific camera called.
   */
  public List<Double> getRobotPositionTimestamps(Integer CAMERA_ID){
    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.getRobotPositionTimestamps();
  }

  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @return List of Poses of the robot since the last control cycle.
   */
  public List<Pose3d> getRobotPositionDeltas(Integer CAMERA_ID){
    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.getRobotPositionDeltas();
  }

  


}
