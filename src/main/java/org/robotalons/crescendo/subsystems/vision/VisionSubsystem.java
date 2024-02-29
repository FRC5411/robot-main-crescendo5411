// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.robotalons.lib.vision.Camera;

import java.io.Closeable;
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
public final class VisionSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static List<Camera> CAMERAS;

  public static PhotonCamera SOURCE;
  public static PhotonCamera SPEAKER_FRONT;
  public static PhotonCamera SPEAKER_REAR;
  public static PhotonCamera INTAKE;

  public static VisionCamera SOURCE_CAMERA;
  public static VisionCamera SPEAKER_FRONT_CAMERA;
  public static VisionCamera SPEAKER_REAR_CAMERA;
  public static VisionCamera INTAKE_CAMERA;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static VisionSubsystem Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Subsystem Constructor.
   */
  public VisionSubsystem(){
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
    CAMERAS.forEach(Camera::update);
  }

  /**
   * Closes Vision Subsystem.
   */
  @Override
  public void close() throws IOException {
    try {
      SOURCE_CAMERA.close();
      SPEAKER_FRONT_CAMERA.close();
      SPEAKER_REAR_CAMERA.close();
      INTAKE_CAMERA.close();
    } catch (final IOException Exception) {}

  }

  /**
   * Takes snapshot from specified Camera integer.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public void preSnapshot(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'preSnapshot' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    CAMERA.preSnapshot();
  }

  /**
   * Takes snapshot from specified Camera integer.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public void postSnapshot(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'postSnapshot' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    CAMERA.postSnapshot();
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class.
   * @return Utility class's instance
   */
  public synchronized VisionSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance)) {
        Instance = new VisionSubsystem();
      }
      return Instance;
  }

  /**
   * Retrieves the Pose3d of the robot that is averaged from two camera estimations.
   * @return New approximation of Pose3d from robot, will return empty if couldnt find two cameras with targets. 
   * @throws IllegalArgumentException when Camera 1 ID or Camera 2 ID is greater than 4, less than 1, or not an integer
   */
  public Optional<Pose3d> getApproximatedRobotPose(){   
    Camera CAMERA1 = CAMERAS.get(0);
    Camera CAMERA2 = CAMERAS.get(1);
    Camera CAMERA3 = CAMERAS.get(2);
    Camera[] CAMERA_2_USE = new Camera[2];

    Optional<Pose3d> PRE_POSE1 = CAMERA1.getRobotPosition();
    Optional<Pose3d> PRE_POSE2 = CAMERA2.getRobotPosition();
    Optional<Pose3d> PRE_POSE3 = CAMERA2.getRobotPosition();

    if(PRE_POSE1.isEmpty()){
      
      if(PRE_POSE2.isEmpty() || PRE_POSE3.isEmpty()){
        return Optional.empty();}
      
        else{
        CAMERA_2_USE[0] = CAMERA2;
        CAMERA_2_USE[1] = CAMERA3;}
    }

    else if(PRE_POSE2.isEmpty()){
      
      if(PRE_POSE1.isEmpty() || PRE_POSE3.isEmpty()){
        return Optional.empty();}
      
        else{
        CAMERA_2_USE[0] = CAMERA1;
        CAMERA_2_USE[1] = CAMERA3;}
    }

    else if(PRE_POSE3.isEmpty()){
      
      if(PRE_POSE1.isEmpty() || PRE_POSE2.isEmpty()){
        return Optional.empty();}
      
        else{
        CAMERA_2_USE[0] = CAMERA1;
        CAMERA_2_USE[1] = CAMERA2;}
    }

    Pose3d CAMERA1_POSE = CAMERA_2_USE[0].getRobotPosition().get();
    Pose3d CAMERA2_POSE = CAMERA_2_USE[1].getRobotPosition().get();

    double avgX = (CAMERA1_POSE.getX() + CAMERA2_POSE.getX()) / 2;
    double avgY = (CAMERA1_POSE.getY() + CAMERA2_POSE.getY()) / 2;
    double avgZ = (CAMERA1_POSE.getZ() + CAMERA2_POSE.getZ()) / 2;

    double avgPitch = (CAMERA1_POSE.getRotation().getX() + CAMERA2_POSE.getRotation().getX()) / 2;
    double avgRoll = (CAMERA1_POSE.getRotation().getY() + CAMERA2_POSE.getRotation().getY()) / 2;
    double avgYaw = (CAMERA1_POSE.getRotation().getZ() + CAMERA2_POSE.getRotation().getZ()) / 2;


    return Optional.of(new Pose3d(avgX, avgY, avgZ, new Rotation3d(avgPitch, avgRoll, avgYaw)));
  }

/**
   * Retrieves the Pose3d of the robot that is averaged from the two camera estimations.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA1_ID that gets the first camera.
   * @param CAMERA2_ID that gets the second camera.
   * @return New approximation of Pose3d from robot. 
   * @throws IllegalArgumentException when Camera 1 ID or Camera 2 ID is greater than 4, less than 1, or not an integer
   */
  public Optional<Pose3d> getApproximatedRobotPose(Integer CAMERA1_ID, Integer CAMERA2_ID){   
    if(CAMERA1_ID > 4 || CAMERA1_ID < 1 ||  Math.floor(CAMERA1_ID) != CAMERA1_ID){
      throw new IllegalArgumentException("Camera ID for method 'getApproximatedRobotPose' should not be greater than 4, less than 1, or not an integer");
    }
    
    if(CAMERA2_ID > 4 || CAMERA2_ID < 1 ||  Math.floor(CAMERA2_ID) != CAMERA2_ID){
      throw new IllegalArgumentException("Camera ID for method 'getApproximatedRobotPose' should not be greater than 4, less than 1, or not an integer");
    }
    
    Camera CAMERA1 = CAMERAS.get(CAMERA1_ID - 1);
    Camera CAMERA2 = CAMERAS.get(CAMERA2_ID - 1);

    Optional<Pose3d> PRE_POSE1 = CAMERA1.getRobotPosition();
    Optional<Pose3d> PRE_POSE2 = CAMERA2.getRobotPosition();
    
    if(PRE_POSE1.isEmpty() || PRE_POSE2.isEmpty()){
      return Optional.empty();
    }

    Pose3d CAMERA1_POSE = PRE_POSE1.get();
    Pose3d CAMERA2_POSE = PRE_POSE2.get();

    double avgX = (CAMERA1_POSE.getX() + CAMERA2_POSE.getX()) / 2;
    double avgY = (CAMERA1_POSE.getY() + CAMERA2_POSE.getY()) / 2;
    double avgZ = (CAMERA1_POSE.getZ() + CAMERA2_POSE.getZ()) / 2;

    double avgPitch = (CAMERA1_POSE.getRotation().getX() + CAMERA2_POSE.getRotation().getX()) / 2;
    double avgRoll = (CAMERA1_POSE.getRotation().getY() + CAMERA2_POSE.getRotation().getY()) / 2;
    double avgYaw = (CAMERA1_POSE.getRotation().getZ() + CAMERA2_POSE.getRotation().getZ()) / 2;


    return Optional.of(new Pose3d(avgX, avgY, avgZ, new Rotation3d(avgPitch, avgRoll, avgYaw)));
  }

/**
   * Retrieves the Pose3d of the robot that is averaged from the two camera estimations.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA1_ID that gets the first camera.
   * @return New approximation of Pose3d from robot. 
   * @throws IllegalArgumentException when Camera 1 ID or Camera 2 ID is greater than 4, less than 1, or not an integer
   */
  public Optional<Pose3d> getApproximatedRobotPose(Integer CAMERA_ID){   
    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'getApproximatedRobotPose' should not be greater than 4, less than 1, or not an integer");
    }
    
    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);

    Optional<Pose3d> PRE_POSE1 = CAMERA.getRobotPosition();
    
    if(PRE_POSE1.isEmpty()){
      return Optional.empty();
    }

    Pose3d POSE = PRE_POSE1.get();


    return Optional.of(POSE);
  }

  /**
   * Provides the robot relative position timestamps of each delta from the last update control cycle up to the current query of specific camera called.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return List of robot relative snapshot time deltas of specific camera called.
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public double[] getRobotPositionTimestamps(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'getRobotPositionTimestamps' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.getRobotPositionTimestamps();
  }

  /**
   * Provides the robot relative (minus offset) position deltas from last update control cycle up to the current query.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return List of Poses of the robot since the last control cycle.
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public Pose3d[] getRobotPositionDeltas(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'snapshot' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.getRobotPositionDeltas();
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation to a known object.
   * @param Target Transformation to a given target anywhere on the field.
   * @return Position of the object relative to the field.
   */
  public Optional<Pose3d> getObjectFieldPose(Transform3d Target){
    return INTAKE_CAMERA.getObjectFieldPose(Target);
  }

  /**
   * Provides the robot relative position to a given object based on the estimated position of this camera and a transformation assuming that.
   * the desired object is the optimal target of this camera.
   * @return Position of the object relative to the field.
   */
  public Optional<Pose3d> getObjectFieldPose(){
    return INTAKE_CAMERA.getObjectFieldPose();
  }

  /**
   * Provides a list of robot-relative transformations to the best target within view of the camera.
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return List of robot-relative target transformations.
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public Optional<Transform3d[]> getTargets(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'getTargets' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);

    return CAMERA.getTargets();
  }

  /**
   * Provides the april tag with the id that we asked for in Pose3d from the specified camera.
   * Fiducial ID : (1-16)
   * @param APRILTAG_ID that gets the specific fiducial marker ID.
   * @return Position of the tag relative to the field.

   */
  public Pose3d getAprilTagPose(Integer APRILTAG_ID){
    return SOURCE_CAMERA.getAprilTagPose(APRILTAG_ID);
  }

  /**
   * Provides a boolean representation of if the module that was specified has an april tag / object detected
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return Boolean if Camera has Target or not
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public boolean hasTargets(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'hasTargets' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.hasTargets();
  }

  /**
   * Provides a number of targets detected by the specified camera (april tag / object detected)
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return Number of Targets found by camera
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public int getNumTargets(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'hasTargets' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);
    return CAMERA.getNumTargets();
  }

  /**
   * Provides the robot-relative transformation to the best target within view of the camera that was called
   * 1 - Source Camera, 2 - Speaker Front Camera, 3- Speaker Rear Camera, 4 - OD Camera.
   * @param CAMERA_ID that gets the specific camera.
   * @return Robot-relative best target transformation
   * @throws IllegalArgumentException when ID is greater than 4, less than 1, or not an integer. 
   */
  public Optional<Transform3d> getOptimalTarget(Integer CAMERA_ID) throws IllegalArgumentException{

    if(CAMERA_ID > 4 || CAMERA_ID < 1 ||  Math.floor(CAMERA_ID) != CAMERA_ID){
      throw new IllegalArgumentException("Camera ID for method 'getOptimalTarget' should not be greater than 4, less than 1, or not an integer");
    }

    Camera CAMERA = CAMERAS.get(CAMERA_ID - 1);

    return CAMERA.getOptimalTarget();
  }

}
