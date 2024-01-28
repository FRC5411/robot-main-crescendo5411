// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotalons.lib.vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
// ---------------------------------------------------------[Photon Vision Module]--------------------------------------------------------//
/**
 *
 *
 * <h1>PhotonVisionModule</h1>
 *
 * <p>Implementation of a single Photon Vision Camera Module, which gathers information on it's position and reports back to the subsystem.</p>
 * 
 * @see Module
 * @see VisionSubsystem
 */
public final class VisionCamera extends Camera {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final PhotonCamera CAMERA;
  private final Transform3d RELATIVE;
  private PhotonPoseEstimator POSE_ESTIMATOR;
  private List<Double> TIMESTAMP_LIST;
  private List<Pose3d> DELTAS_LIST;
  private List<Pose3d> POSES_LIST;

  
  private final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  public VisionCamera(final PhotonCamera Camera, final Transform3d Relative, final String Identifier) {
    super(Camera.getCameraTable(), Relative, Identifier);
    CAMERA = Camera;
    RELATIVE = Relative;

    POSE_ESTIMATOR = new PhotonPoseEstimator(
      FIELD_LAYOUT, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      CAMERA,
      RELATIVE);

    TIMESTAMP_LIST = new ArrayList<>();
    DELTAS_LIST = new ArrayList<>();
    POSES_LIST = new ArrayList<>();
  }

  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  @Override
  public void periodic(){}

  @Override
  public void update() {
    POSE_ESTIMATOR = new PhotonPoseEstimator(
      FIELD_LAYOUT, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      CAMERA,
      RELATIVE);
  }

  @Override
  public void snapshot() {
    CAMERA.takeOutputSnapshot();
  }

  @Override
  public void close() throws IOException {
    // CHECK
    POSE_ESTIMATOR = null;
    TIMESTAMP_LIST = null;
    DELTAS_LIST = null;
    POSES_LIST = null;
  }

  @Override
  public void set(VisionLEDMode Mode){
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public void set(Integer Mode) {
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public Matrix<Num, N1> getStandardDeviations() {
    // TODO Fix this method to be working //
    Pose3d POSE = new Pose3d();
    int length = POSES_LIST.size();

    double[] x = new double[length];
    double[] y = new double[length];
    double[] z = new double[length];
    double[] pitch = new double[length];
    double[] roll = new double[length];
    double[] yaw = new double[length];  


    // Return Standard Deviation of x, y, x, pitch, roll, yaw
    for(int i = 0; i < POSES_LIST.size(); i++){
      POSE = POSES_LIST.get(i);

      x[i] = POSE.getX();
      y[i] = POSE.getY();
      z[i] = POSE.getZ();
      pitch[i] = POSE.getRotation().getX();
      roll[i] = POSE.getRotation().getY();
      yaw[i] = POSE.getRotation().getZ();
    }

    double sdX = getSD(x);
    double sdY = getSD(y);
    double sdZ = getSD(z);
    double sdPitch = getSD(pitch);
    double sdRoll = getSD(roll);
    double sdYaw = getSD(yaw);
    return null;
  }

  private double getSD(double[] nums){
    double mean = 0, sumSquareDiff = 0;


    for(double num : nums){
      mean += num;
    }

    mean /= nums.length;

    for(double num : nums){
      sumSquareDiff += Math.pow(num - mean, 2);
    }

    return Math.sqrt(sumSquareDiff / nums.length - 1);
  }

  @Override
  public Optional<Matrix<N3, N3>> getCameraMatrix() {
    return CAMERA.getCameraMatrix();
  }

  @Override
  public Optional<Matrix<N5, N1>> getCoefficientMatrix() {
    return CAMERA.getDistCoeffs();
  }

  @Override
  public Pair<Integer, Integer> getImageResolution() {
    throw new UnsupportedOperationException("Unimplemented method 'getImageResolution'");
  }

  @Override
  public List<Double> getRobotPositionTimestamps() {
    return TIMESTAMP_LIST;
  }

  @Override
  public List<Pose3d> getRobotPositionDeltas() {

    
    Pose3d PREV_POSE = POSES_LIST.get(0);
    Pose3d CURRENT_POSE = POSES_LIST.get(1);

    Pose3d DELTA = new Pose3d();

    for(int i =  0; i < POSES_LIST.size(); i++){
      PREV_POSE = POSES_LIST.get(i);
      CURRENT_POSE = POSES_LIST.get(i+1);

      double D_X = CURRENT_POSE.getX() - PREV_POSE.getX();
      double D_Y = CURRENT_POSE.getY() - PREV_POSE.getY();
      double D_Z = CURRENT_POSE.getZ() - PREV_POSE.getZ();
      double D_PITCH = CURRENT_POSE.getRotation().getX() - PREV_POSE.getRotation().getX();
      double D_ROLL = CURRENT_POSE.getRotation().getY() - PREV_POSE.getRotation().getY();
      double D_YAW = CURRENT_POSE.getRotation().getZ() - PREV_POSE.getRotation().getZ();
      
      DELTA = new Pose3d(D_X, D_Y, D_Z, new Rotation3d(D_PITCH, D_ROLL, D_YAW));
      DELTAS_LIST.add(DELTA);
    }

    return DELTAS_LIST;

  }

  @Override
  public Pose3d getRobotPosition() {
    EstimatedRobotPose CURRENT_POSE = POSE_ESTIMATOR.update().get();
    
    Pose3d CONVERTED = CURRENT_POSE.estimatedPose;
    POSES_LIST.add(CONVERTED);

    double time = CURRENT_POSE.timestampSeconds;
    TIMESTAMP_LIST.add(time);

    return CONVERTED;
  }

  @Override
  public List<Transform3d> getTargets() {
    return CAMERA.getLatestResult().getTargets().stream().map(PhotonTrackedTarget::getBestCameraToTarget).toList();
  }

  @Override
  public Transform3d getOptimalTarget() {
    return CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
}
