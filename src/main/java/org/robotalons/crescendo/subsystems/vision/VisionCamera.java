// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.robotalons.crescendo.subsystems.vision.Constants.Measurements;
import org.robotalons.lib.vision.Camera;

import java.io.IOException;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
  private List<Pose3d> POSES_LIST;
  private int COUNTER;

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  public VisionCamera(final PhotonCamera Camera, final Transform3d Relative, final String Identifier) {
    super(Camera.getCameraTable(), Relative, Identifier);
    CAMERA = Camera;
    RELATIVE = Relative;

    POSE_ESTIMATOR = new PhotonPoseEstimator(
      Measurements.FIELD_LAYOUT, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      CAMERA,
      RELATIVE);

    TIMESTAMP_LIST = new ArrayList<>();
    POSES_LIST = new ArrayList<>();

    COUNTER = 0;
  }

  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  @Override
  public void periodic(){}

  @Override
  public void update() {
    POSE_ESTIMATOR = new PhotonPoseEstimator(
      Measurements.FIELD_LAYOUT, 
      PoseStrategy.CLOSEST_TO_LAST_POSE, 
      CAMERA,
      RELATIVE);
  }

  @Override
  public void preSnapshot() {
    CAMERA.takeInputSnapshot();
  }

  @Override
  public void postSnapshot() {
    CAMERA.takeOutputSnapshot();
  }

  @Override
  public void close() throws IOException {
    POSE_ESTIMATOR = null;
    TIMESTAMP_LIST.clear();
    POSES_LIST.clear();
    CAMERA.close();
    COUNTER = 0;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  @Override
  public Matrix<Num, N1> getStandardDeviations() {
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

    double sdX = getStandardDeviation(x);
    double sdY = getStandardDeviation(y);
    double sdZ = getStandardDeviation(z);
    double sdPitch = getStandardDeviation(pitch);
    double sdRoll = getStandardDeviation(roll);
    double sdYaw = getStandardDeviation(yaw);

    double[] data = new double[]{sdX, sdY, sdZ, sdPitch, sdRoll, sdYaw};

    return MatBuilder.fill(() -> 6, Nat.N1(), data);
  }
 
  private double getStandardDeviation(double[] nums){
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
  public double[] getRobotPositionTimestamps() {
    double[] timestamps = new double[TIMESTAMP_LIST.size()];

    for(int i = 0; i < timestamps.length; i++){
      timestamps[i] = TIMESTAMP_LIST.get(i);
    }

    return timestamps;
  }

  @Override
  public Pose3d[] getRobotPositionDeltas() {

    Pose3d[] DELTAS_LIST = new Pose3d[POSES_LIST.size() - 1];
    
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
      DELTAS_LIST[i] = DELTA;
    }

    return DELTAS_LIST;

  }

  @Override
  public Optional<Pose3d> getRobotPosition() {

    if(POSE_ESTIMATOR.update().isEmpty()){
      return Optional.empty();
    }


    if(COUNTER == 0){
      EstimatedRobotPose POSE = POSE_ESTIMATOR.update().get();
      
      Pose3d CONVERTED = POSE.estimatedPose;
      POSES_LIST.add(CONVERTED);

      double time = POSE.timestampSeconds;
      TIMESTAMP_LIST.add(time);

      COUNTER ++;

      return Optional.of(CONVERTED);
    }

    else{
      Pose3d PREV_POSE = POSES_LIST.get(COUNTER);
      POSE_ESTIMATOR.setLastPose(PREV_POSE);

      EstimatedRobotPose CURRENT_POSE = POSE_ESTIMATOR.update().get();
      
      Pose3d CONVERTED = CURRENT_POSE.estimatedPose;
      POSES_LIST.add(CONVERTED);

      double time = CURRENT_POSE.timestampSeconds;
      TIMESTAMP_LIST.add(time);

      COUNTER ++;

      return Optional.of(CONVERTED);
    }

  }

  @Override
  public Optional<Pose3d> getObjectFieldPose() {
    Optional<Pose3d> ROBOT = getRobotPosition();
    

    if(ROBOT.isEmpty() || getOptimalTarget().isEmpty()){
      return Optional.empty();
    }

    Transform3d TARGET = getOptimalTarget().get();

    return Optional.of(new Pose3d(
      (TARGET.getX() + ROBOT.get().getX()),
      (TARGET.getY() + ROBOT.get().getY()),
      TARGET.getZ(),
      new Rotation3d()
      ));
  }

  @Override
  public Optional<Pose3d> getObjectFieldPose(Transform3d TARGET) {
    Optional<Pose3d> ROBOT = getRobotPosition();

    if(ROBOT.isEmpty()){
      return Optional.empty();
    }

    return Optional.of(new Pose3d(
      (TARGET.getX() + ROBOT.get().getX()),
      (TARGET.getY() + ROBOT.get().getY()),
      TARGET.getZ(),
      new Rotation3d()
    ));
  }

  @Override
  public Optional<Transform3d[]> getTargets() {

    if(CAMERA.getLatestResult().getTargets().isEmpty()){
      return Optional.empty();
    }
    
    ArrayList<Transform3d> targets = (ArrayList<Transform3d>) CAMERA.getLatestResult().getTargets().stream().map(PhotonTrackedTarget::getBestCameraToTarget).toList();
    Transform3d[] transforms = new Transform3d[targets.size()];

    for(int i = 0; i < transforms.length; i++){
      transforms[i] = targets.get(i);
    }

    return Optional.of(transforms);
  }

  @Override
  public Pose3d getAprilTagPose(Integer FIDICUAL_ID) {
    return Measurements.FIELD_LAYOUT.getTagPose(FIDICUAL_ID).get();
  }

  @Override
  public boolean hasTargets(){
    return CAMERA.getLatestResult().hasTargets();
  }

  @Override
  public int getNumTargets(){
    return CAMERA.getLatestResult().getTargets().size();
  }


  @Override
  public Optional<Transform3d> getOptimalTarget() {
    if(CAMERA.getLatestResult().getBestTarget() == null){
      return Optional.empty();
    }
    return Optional.of(CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget());
  }

  @Override
  public Double getLatency(){
    return CAMERA.getLatestResult().getLatencyMillis();
  }

  @Override
  public Boolean getConnected(){
    return CAMERA.isConnected();
  }

  @Override
  public String getName(){
    return CAMERA.getName();
  }
}
