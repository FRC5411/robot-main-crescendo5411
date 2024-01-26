// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.io.IOException;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// -----------------------------------------------------------------[Camera]----------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the odometry and kinematics of a given robot.
 * 
 */
public abstract class Camera implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  protected final CameraStatusContainer STATUS = new CameraStatusContainer();
  protected final NetworkTableInstance INSTANCE;
  protected final Transform3d OFFSET;
  protected final String IDENTITY;
  // -------------------------------------------------------------[Constructors]------------------------------------------------------------//
  /**
   * Camera Constructor
   * @param Instance   Network table instance for raw object data to be retrieved from
   * @param Relative   Robot relative offset of the camera from the center of the robot's base
   * @param Identifier Name of this camera, not bound by the type of camera it implements upon
   */
  protected Camera(final NetworkTableInstance Instance, final Transform3d Relative, final String Identifier) {
    INSTANCE = Instance;
    OFFSET = Relative;
    IDENTITY = Identifier;
  }
  // ---------------------------------------------------------------[Internal]--------------------------------------------------------------//
  /**
   * Describes the mode of the camera's LEDs, which may affect the picture quality of the camera
   */
  public enum VisionMode {
    BLINK,
    ON,
    OFF,
    DEFAULT,
  }

  /**
   * Describes the mode of the camera's pipelines, which affect how object data is captured and processed by the camera
   */
  public enum PipelineMode {
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX,
    SEVEN,
    EIGHT,
    NINE
  }
  // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * This method is called periodically by the {@link CommandScheduler}. Useful for updating
   * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
   * to be consistent within their own codebases about which responsibilities will be handled by
   * Commands, and which will be handled here.
   */
  public abstract void periodic();
  /**
   * Updates the underlying signals within this module.
   */
  public abstract void update();

  /**
   * Forces the camera to take a snapshot of it's current processed feed.
   */
  public abstract void snapshot();

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
  /**
   * Mutates the underlying VisionMode state of the relevant camera
   * @param Mode New vision mode of camera
   */
  public abstract void set(final VisionMode Mode);

  /**
   * Mutates the underlying PipelineMode state of the relevant camera
   * @param Mode New pipeline mode of camera
   */
  public abstract void set(final PipelineMode Mode);
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * @return List of Poses of the robot since the last control cycle
   */
  public abstract List<Pose3d> getRobotPositionDeltas();

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query.
   * @return List of robot relative snapshot time deltas
   */
  public abstract List<Double> getRobotPositionTimestamps();

  /**
   * Provides the robot relative (minus offset) position immediately.
   * @return Current estimated Pose of the robot at this moment
   */
  public abstract Pose3d getRobotPosition();

  /**
   * Provides the offset of this camera relative to the center of the robot.
   * @return Permanent offset of the robot relative to the center of the robot
   */
  public Transform3d getRobotOffset() {
    return OFFSET;
  }

  /**
   * Provides the identifier name of this camera, which is separate from the actual name on network tables
   * @return String representation of the camera's name
   */
  public String getName() {
    return IDENTITY;
  }

  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */
  public Boolean getConnected() {
    return STATUS.Connected;
  }
}
