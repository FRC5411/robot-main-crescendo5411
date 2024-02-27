// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.Closeable;
import java.io.IOException;
import java.util.Optional;
// -----------------------------------------------------------------[Camera]----------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the odometry and kinematics of a given robot.
 * 
 */
public abstract class Camera implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final CameraStatusContainer CAMERA_STATUS = new CameraStatusContainer();
  protected final TargetStatusContainer TARGET_STATUS = new TargetStatusContainer();
  protected final NetworkTable INSTANCE;
  protected final Transform3d OFFSET;
  protected final String IDENTITY;
  // -------------------------------------------------------------[Constructors]------------------------------------------------------------//
  /**
   * Camera Constructor
   * @param Instance   Network table instance for raw object data to be retrieved from
   * @param Relative   Robot relative offset of the camera from the center of the robot's base
   * @param Identifier Name of this camera, not bound by the type of camera it implements upon
   */
  protected Camera(final NetworkTable Instance, final Transform3d Relative, final String Identifier) {
    INSTANCE = Instance;
    OFFSET = Relative;
    IDENTITY = Identifier;
  }
  // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
   * to be consistent within their own codebase about which responsibilities will be handled by
   * Commands, and which will be handled here.
   */
  public abstract void periodic();

  /**
   * Updates the underlying signals within this module
   */
  public abstract void update();

  /**
   * Forces the camera to take a preprocessed snapshot of it's current processed feed.
   */
  public abstract void preSnapshot();

  /**
   * Forces the camera to take a preprocessed snapshot of it's current processed feed.
   */
  public abstract void postSnapshot();

  /**
   * Requests raw object metadata from the raw network table instance of the camera
   * @param Topic                 Name of the requested topic as a string value
   * @return                      Value of the requested topic as an object, which must be cast to the corresponding datatype
   * @throws NullPointerException When the value requested does not exist within the network tables instance
   */
  public synchronized Object request(final String Topic) throws NullPointerException {
    synchronized(INSTANCE) {
      return INSTANCE.getTopic(Topic);
    }
  }

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  public abstract void close() throws IOException;
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation to a known object
   * @param Target Transformation to a given target anywhere on the field
   * @return Position of the object relative to the field
   */
  public abstract Optional<Pose3d> getObjectFieldPose(final Transform3d Target);

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation assuming that
   * the desired object is the optimal target of this camera.
   * @return Position of the object relative to the field
   */
  public abstract Optional<Pose3d> getObjectFieldPose();

  /**
   * Provides the april tag with the id that we asked for in Pose3d
   * @return Position of the tag relative to the field
   */
  public abstract Pose3d getAprilTagPose(final Integer ID);

  /**
   * Provides the confidence or standard deviation of the cameras evaluations of estimations
   * @return Matrix of standard deviations of estimation
   */
  public abstract Matrix<N3,N1> getStandardDeviations();

  /**
   * Provides the matrix of this camera's hardware data, distortion, etc.
   * @return Optional value that may exist of the camera's hardware data
   */
  public abstract Optional<Matrix<N3, N3>> getCameraMatrix();

  /**
   * Provides the coefficient matrix of the distance camera relative to the current target
   * @return Optional value that may exist of the camera's distance coefficients
   */
  public abstract Optional<Matrix<N5, N1>> getCoefficientMatrix();

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query.
   * @return List of robot relative snapshot time deltas
   */
  public double[] getRobotPositionTimestamps(){
    return CAMERA_STATUS.Timestamps;
  }

  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * @return List of Poses of the robot since the last control cycle
   */
  public Pose3d[] getRobotPositionDeltas(){
    return CAMERA_STATUS.Deltas;
  };

  /**
   * Provides the robot relative (minus offset) position immediately.
   * @return Current estimated Pose of the robot at this moment
   */
  public abstract Optional<Pose3d> getRobotPosition();

  /**
   * Provides the offset of this camera relative to the center of the robot.
   * @return Permanent offset of the robot relative to the center of the robot
   */
  public Transform3d getRobotOffset() {
    return OFFSET;
  }

  /**
   * Provides a list of robot-relative transformations to the best target within view of the camera
   * @return List of robot-relative target transformations
   */
  public Transform3d[] getTargets(){
    return TARGET_STATUS.Targets;
  }

  /**
   * Provides a boolean representation of if the module has an april tag / object detected
   * @return List of robot-relative target transformations
   */
  public boolean hasTargets(){
    return TARGET_STATUS.HasTargets;
  }


  /**
   * Provides the robot-relative transformation to the best target within view of the camera
   * @return Robot-relative best target transformation
   */
  public Transform3d getOptimalTarget(){
    return TARGET_STATUS.OptimalTransform;
  }

  /**
   * Provides the number of targets that is detected within the view of the camera
   * @return Number of targets detected by camera
   */
  public int getNumTargets(){
    return TARGET_STATUS.TotalTargets;
  }

  /**
   * Provides the identifier name of this camera, which is separate from the actual name on network tables
   * @return String representation of the camera's name
   */
  public String getName() {
    return CAMERA_STATUS.Name;
  }

  /**
   * Provides the latency or 'lag' of the camera to retrieve information, should always be greater than 0d.
   * @return Double representation of the latency to retrieve information
   */
  public Double getLatency() {
    return CAMERA_STATUS.Latency;
  }

  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */
  public Boolean getConnected() {
    return CAMERA_STATUS.Connected;
  }
}
