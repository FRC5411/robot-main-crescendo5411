// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotalons.crescendo.subsystems.vision.Constants;
import org.robotalons.crescendo.subsystems.vision.Constants.Vision;

import java.io.Closeable;
import java.io.IOException;
import java.util.List;

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
  private final PhotonCamera CAMERA;
  private final double[] CAMERAPOSE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  /**
   * Updates the underlying signals within this module.
   */

  // ---------------------------------------------------------------[Constructor]----------------------------------------------------------------//
  
  public Camera(String CameraName){
    CAMERA = new PhotonCamera(CameraName);

    if(CameraName.equals("Camera_1")){
      CAMERAPOSE = Vision.CAMERA_1_POSE;
    }

    else if(CameraName.equals("Camera_2")){
      CAMERAPOSE = Vision.CAMERA_2_POSE;
    }

    else if(CameraName.equals("Camera_3")){
      CAMERAPOSE = Vision.CAMERA_3_POSE;
    }

    else{
      CAMERAPOSE = Vision.CAMERA_4_POSE;
    }

  }

  public abstract void update();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   */
  public abstract void close() throws IOException;

  // ---------------------------------------------------------------[Accessors]----------------------------------------------------------------//
  public double getLatency(){
    return CAMERA.getLatestResult().getLatencyMillis();
  }

  public PhotonTrackedTarget getBestTarget(){
    return CAMERA.getLatestResult().getBestTarget();
  }

  public boolean hasTarget(int ID){
    List<PhotonTrackedTarget> targets = getTargets();

    for(PhotonTrackedTarget target : targets){
      if(target.getFiducialId() == ID){
        return true;
      }
    }

    return false;
  }

  public List<PhotonTrackedTarget> getTargets(){
    return CAMERA.getLatestResult().getTargets();
  }

  public Pose3d getAprilTagPose(int fiducialID){
    return Constants.AprilTag.APRIL_TAGS[fiducialID - 1];
  }

  public Pose3d getRobotPose(){
    PhotonTrackedTarget target = getBestTarget();
    int marker = target.getFiducialId();
    
    Transform3d cameraTorobot = new Transform3d(
      new Translation3d(CAMERAPOSE[0], CAMERAPOSE[1], CAMERAPOSE[2]),
      new Rotation3d(CAMERAPOSE[3], CAMERAPOSE[4], CAMERAPOSE[5])
    );
    
    return PhotonUtils.estimateFieldToRobotAprilTag(
      target.getBestCameraToTarget(), 
      getAprilTagPose(marker),
      cameraTorobot);
  }


  public Transform3d getTransformationToObject(){
    return CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget();
  }

  public Transform3d getTransformationToObject(PhotonTrackedTarget target){
    return target.getBestCameraToTarget();
  }


  // Use for Note Detection //
  // TODO: Check logic with Cody //
  public Pose3d getObjectFieldPose(){
    Pose3d bot = getRobotPose();
    Transform3d transformation = getTransformationToObject();

    double x = transformation.getX();
    double y = transformation.getY();

    double botx = bot.getX();
    double boty = bot.getY();

    Pose3d targetPose = new Pose3d(x + botx , y + boty, 0, new Rotation3d(0, 0, 0));
    return targetPose;
  }

  public Pose3d getObjectFieldPose(PhotonTrackedTarget target){
    Pose3d bot = getRobotPose();
    Transform3d transformation = getTransformationToObject(target);

    double x = transformation.getX();
    double y = transformation.getY();

    double botx = bot.getX();
    double boty = bot.getY();

    Pose3d targetPose = new Pose3d(x + botx , y + boty, 0, new Rotation3d(0, 0, 0));
    return targetPose;
  }


  /**
   * Provides a boolean representation of if the module is still connected to the system and all signals are okay.
   * @return Boolean representing Connectivity
   */

  // TODO: Change made in code tell Cody //
  public Boolean getConnected() {
    return CAMERA.isConnected();
    //return STATUS.Connected;
  }
}
