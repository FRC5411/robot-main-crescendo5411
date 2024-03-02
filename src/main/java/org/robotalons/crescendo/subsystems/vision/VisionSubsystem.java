// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.robotalons.crescendo.subsystems.vision.Constants.Ports;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.vision.Camera;

import java.util.concurrent.atomic.AtomicInteger;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

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
public final class VisionSubsystem extends TalonSubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static List<Camera> CAMERAS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static VisionSubsystem Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Subsystem Constructor.
   */
  public VisionSubsystem() {
    super(("Vision Subsystem"));
  } static {
    CAMERAS = List.of(
      new VisionCamera(
        new PhotonCamera(Ports.SOURCE_CAMERA_NAME), 
        Constants.Measurements.SOURCE_CAMERA_POSE
      ),
      new VisionCamera(
        new PhotonCamera(Ports.FRONT_RIGHT_CAMERA_NAME), 
        Constants.Measurements.SPEAKER_FRONT_CAMERA_POSE
      ),
      new VisionCamera(
        new PhotonCamera(Ports.REAR_LEFT_CAMERA_NAME), 
        Constants.Measurements.SPEAKER_REAR_CAMERA_POSE
      ),
      new VisionCamera(
        new PhotonCamera(Ports.REAR_RIGHT_CAMERA_NAME), 
        Constants.Measurements.INTAKE_CAMERA_POSE
    )); 
  }
  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    CAMERAS.forEach((Camera) -> {
      if (Camera.getConnected()) {
        Camera.periodic();
      }
    });
  }

  @Override
  public void close() {
    CAMERAS.forEach((Camera) -> {
      try {
        Camera.close();
      } catch(final IOException Ignored) {}
    });
  }

  /**
   * Takes snapshot from specified Camera integer.
   * @param Identifier Camera identifier to query from.
   */
  public static void snapshotInput(final CameraType Identifier) {
    CAMERAS.get(Identifier.getValue()).snapshotInput();
  }

  /**
   * Takes snapshot from specified Camera integer.
   * @param Identifier Camera identifier to query from.
   */
  public static void snapshotOutput(final CameraType Identifier) {
    CAMERAS.get(Identifier.getValue()).snapshotOutput();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes a queried camera type with an integer value in a more human-parsable format
   */
  public enum CameraType {
    SOURCE_CAMERA((0)),
    SPEAKER_FRONT_CAMERA((1)),
    SPEAKER_REAR_CAMERA((2)),
    INTAKE_CAMERA((3));

    private final Integer Value;

    CameraType(final Integer Value) {
      this.Value = Value;
    }

    /**
     * Provides the actual camera indexable value of this enum type
     * @return Integer format of this camera number
     */
    public Integer getValue() {
      return this.Value;
    }
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class.
   * @return Utility class's instance
   */
  public static synchronized TalonSubsystemBase getInstance() {
    if (java.util.Objects.isNull(Instance)) {
      Instance = new VisionSubsystem();
    }
    return Instance;
  }

  /**
   * Retrieves the Pose3d of the robot that is averaged from the all camera estimations.
   * @return New approximation of Pose3d from robot. 
   */
  public static Optional<Pose3d> getApproximatedRobotPose(){   
    final Pose3d EstimatedPoseAverage = new Pose3d();
    final AtomicInteger ValidPoseCount = new AtomicInteger();
    CAMERAS.forEach((Camera) -> {
      Camera.getRobotPosition().ifPresent((Pose) -> {
        EstimatedPoseAverage.plus(new Transform3d(Pose.getTranslation(), Pose.getRotation()));
        ValidPoseCount.incrementAndGet();
      });
    });
    return Optional.ofNullable(EstimatedPoseAverage != new Pose3d()? EstimatedPoseAverage.div(ValidPoseCount.get()): null);
  }

  /**
   * Retrieves the Pose3d of the robot that is averaged from the two camera estimations.
   * @param Identifier Camera identifier to query from.
   * @return New approximation of Pose3d from robot.
   */
  public static Optional<Pose3d> getApproximatedRobotPose(final CameraType Identifier){       
    return CAMERAS.get(Identifier.getValue()).getRobotPosition();
  }

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query of specific camera called.
   * @param Identifier Camera identifier to query from.
   * @return List of robot relative snapshot time deltas of specific camera called.
   */
  public static double[] getRobotPositionTimestamps(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).getRobotPositionTimestamps();
  }

  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * @param Identifier Camera identifier to query from.
   * @return List of Poses of the robot since the last control cycle.
   */
  public static Pose3d[] getRobotPositionDeltas(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).getRobotPositionDeltas();
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation to a known object.
   * @param Target Transformation to a given target anywhere on the field.
   * @return Position of the object relative to the field.
   */
  public static Optional<Pose3d> getObjectFieldPose(final Transform3d Target){
    return CAMERAS.get(CameraType.INTAKE_CAMERA.getValue()).getObjectFieldPose(Target);
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation assuming that.
   * the desired object is the optimal target of this camera.
   * @return Position of the object relative to the field.
   */
  public static Optional<Pose3d> getObjectFieldPose(){
    return CAMERAS.get(CameraType.INTAKE_CAMERA.getValue()).getObjectFieldPose();
  }

  /**
   * Provides a list of robot-relative transformations to the best target within view of the camera.
   * @param Identifier Camera identifier to query from.
   * @return List of robot-relative target transformations.
   */
  @SuppressWarnings("unchecked")
  public static Optional<Transform3d>[] getTargets(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).getTargets().toArray(Optional[]::new);
  }

  /**
   * Provides the april tag with the id that we asked for in Pose3d from the specified camera.
   * Fiducial ID : (1-16)
   * @param Identifier Camera identifier to query from.
   * @param Tag        ID of the april tag
   * @return Position of the tag relative to the field.
   */
  public static Pose3d getAprilTagPose(final CameraType Identifier, final Integer Tag){
    return CAMERAS.get(Identifier.getValue()).getAprilTagPose(Tag);
  }

  /**
   * Provides a boolean representation of if the module that was specified has an april tag / object detected
   * @param Identifier Camera identifier to query from.
   * @return Boolean if Camera has Target or not
   */
  public static Boolean hasTargets(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).hasTargets();
  }

  /**
   * Provides a number of targets detected by the specified camera (april tag / object detected)
   * @param Identifier Camera identifier to query from.
   * @return Number of Targets found by camera
   */
  public static Integer getNumTargets(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).getNumTargets();
  }

  /**
   * Provides the robot-relative transformation to the best target within view of the camera that was called
   * @param Identifier Camera identifier to query from.
   * @return Robot-relative best target transformation
   */
  public static Optional<Transform3d> getOptimalTarget(final CameraType Identifier) {
    return CAMERAS.get(Identifier.getValue()).getOptimalTarget();
  }

}