// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N0;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.vision.Constants.Devices;
import org.robotalons.crescendo.subsystems.vision.Constants.Measurements;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.vision.Camera;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

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
public final class VisionSubsystem extends TalonSubsystemBase<Keybindings,Preferences, N0> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<Camera> CAMERAS;
  public static final List<CameraIdentifier> ALL_CAMERA_IDENTIFIERS = List.of(
    CameraIdentifier.SPEAKER_LEFT_CAMERA,
    CameraIdentifier.SPEAKER_RIGHT_CAMERA,
    CameraIdentifier.INTAKE_CAMERA
  );  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static VisionSubsystem Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Subsystem Constructor.
   */
  public VisionSubsystem() {
    super(("Vision Subsystem"), () -> (0));
  } static {
    CAMERAS = List.of(
      Devices.FRONT_LEFT_CAMERA,
      Devices.FRONT_RIGHT_CAMERA,
      Devices.REAR_RIGHT_CAMERA
    ); 
  }
  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCK.lock();
    CAMERAS.parallelStream().forEach((Camera) -> {
      if(Camera.getConnected()) {
        Camera.periodic();
      }
    });
    Constants.Objects.ODOMETRY_LOCK.unlock();
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
  public static void snapshotInput(final CameraIdentifier Identifier) {
    CAMERAS.get(Identifier.getValue()).snapshotInput();
  }

  /**
   * Takes snapshot from specified Camera integer.
   * @param Identifier Camera identifier to query from.
   */
  public static void snapshotOutput(final CameraIdentifier Identifier) {
    CAMERAS.get(Identifier.getValue()).snapshotOutput();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes a queried camera type with an integer value in a more human-parsable format
   */
  public enum CameraIdentifier {
    SPEAKER_LEFT_CAMERA((0)),
    SPEAKER_RIGHT_CAMERA((1)),
    INTAKE_CAMERA((2));

    private final Integer Value;

    CameraIdentifier(final Integer Value) {
      this.Value = Value;
    }

    /**
     * Provides the actual camera indexable value of this enum type can be used interchangeably with {@link #ordinal()}.
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
  public static synchronized TalonSubsystemBase<Keybindings,Preferences,N0> getInstance() {
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
    CAMERAS.forEach((Camera) -> Camera.getRobotPosition().ifPresent((Pose) -> {
      EstimatedPoseAverage.plus(new Transform3d(Pose.getTranslation(), Pose.getRotation()));
      ValidPoseCount.incrementAndGet();
    }));
    return Optional.ofNullable(!EstimatedPoseAverage.equals(new Pose3d()) && ValidPoseCount.get() > 0? EstimatedPoseAverage.div(ValidPoseCount.get()): null);
  }

  /**
   * Retrieves the Pose3d of the robot that is averaged from the two camera estimations.
   * @param Identifier Camera identifier to query from.
   * @return New approximation of Pose3d from robot.
   */
  public static Optional<Pose3d> getApproximatedRobotPose(final CameraIdentifier Identifier){       
    return CAMERAS.get(Identifier.getValue()).getRobotPosition();
  }

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query of specific camera called.
   * @param Identifier Camera identifier to query from.
   * @return Array of robot relative snapshot time deltas of specific camera called.
   */
  public static double[] getRobotPositionTimestamps(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getRobotPositionTimestamps();
  }

  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * @param Identifier Camera identifier to query from.
   * @return Array of Poses of the robot since the last control cycle.
   */
  public static Pose2d[] getRobotPositionDeltas(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getRobotPositionDeltas();
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation to a known object.
   * @param Target Transformation to a given target anywhere on the field.
   * @return Position of the object relative to the field.
   */
   public static Optional<Pose2d> getObjectFieldPose(final Transform3d Target){
     return CAMERAS.get(CameraIdentifier.INTAKE_CAMERA.getValue()).getObjectFieldPose(new Transform2d(Target.getTranslation().toTranslation2d(), Target.getRotation().toRotation2d()));
   }

  /**
   * Provides the values of standard deviations of the most recent object detection result on the camera
   * @param Identifier Camera identifier to query from.
   * @return Standard Deviations of the recorded object detection values
   */
  public static Matrix<N3,N1> getStandardDeviations(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getStandardDeviations();
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation assuming that.
   * the desired object is the optimal target of this camera.
   * @return Position of the object relative to the field.
   */
  public static Optional<Pose2d> getObjectFieldPose(){
    return CAMERAS.get(CameraIdentifier.INTAKE_CAMERA.getValue()).getObjectFieldPose();
  }

  /**
   * Provides a list of robot-relative transformations to the best target within view of the camera.
   * @param Identifier Camera identifier to query from.
   * @return Array of robot-relative target transformations.
   */
  @SuppressWarnings("unchecked")
  public static Optional<Transform3d>[] getTargets(final CameraIdentifier Identifier) {
    return (Optional<Transform3d>[]) CAMERAS.get(Identifier.getValue()).getTargets().toArray(Optional[]::new);
  }

  /**
   * Provides the april tag with the id that we asked for in Pose3d from the specified camera.
   * Fiducial ID : (1-16)
   * @param Tag ID of the april tag
   * @return Position of the tag relative to the field.
   */
  public static Optional<Pose3d> getAprilTagPose(final Integer Tag){
    return Measurements.FIELD_LAYOUT.getTagPose(Tag);
  }

  /**
   * Provides the robot offset of a given camera robot-relative
   * @param Identifier Camera identifier to query from.
   * @return Transformation from the origin (robot) to a given camera
   */
  public static Transform3d getCameraTransform(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getRobotOffset();
  }

  /**
   * Provides the 'most optimal' target transformation between a list of cameras, compared using the normalization of their respective transformations
   * @param Cameras List of cameras to take data from, any copies of cameras identifiers are removed
   * @return if possible, 'most optimal' target transformation 
   */
  public static Optional<Transform2d> getOptimalTarget(List<CameraIdentifier> Cameras) {
    Cameras = Cameras.stream().distinct().toList();
    Optional<Transform2d> Optimal = Optional.of(new Transform2d());
    for(final var Identifier: Cameras) {
      final var RelativeOptimal = CAMERAS.get(Identifier.getValue()).getOptimalTarget();
      if(RelativeOptimal.isPresent()) {
        if(Optimal.get().getTranslation().getNorm() < RelativeOptimal.get().getTranslation().getNorm()) {
          Optimal = RelativeOptimal;
        }
      }      
    }
    return Optimal;
  }

  /**
   * Provides the 'most optimal' target transformation between all cameras, compared using the normalization of their respective transformations
   * @return if possible, 'most optimal' target transformation 
   */
  public static Optional<Transform2d> getOptimalTarget() {
    return getOptimalTarget(List.of(CameraIdentifier.values()));
  }

  /**
   * Provides a boolean representation of if the module that was specified has an april tag / object detected
   * @param Identifier Camera identifier to query from.
   * @return Boolean if Camera has Target or not
   */
  public static Boolean hasTargets(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).hasTargets();
  }

  /**
   * Provides a number of targets detected by the specified camera (april tag / object detected)
   * @param Identifier Camera identifier to query from.
   * @return Number of Targets found by camera
   */
  public static Integer getNumTargets(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getTargetCount();
  }

  /**
   * Provides the robot-relative transformation to the best target within view of the camera that was called
   * @param Identifier Camera identifier to query from.
   * @return Robot-relative best target transformation
   */
  public static Optional<Transform2d> getOptimalTarget(final CameraIdentifier Identifier) {
    return CAMERAS.get(Identifier.getValue()).getOptimalTarget();
  }

}