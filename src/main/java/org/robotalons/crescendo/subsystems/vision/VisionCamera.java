// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotalons.crescendo.subsystems.vision.Constants.Measurements;
import org.robotalons.lib.vision.Camera;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
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
  private final PhotonPoseEstimator POSE_ESTIMATOR;
  private final PhotonCamera CAMERA;
  private final Transform3d RELATIVE;  
  private final List<Double> TIMESTAMPS;
  private final List<Pose3d> POSES;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Camera Constructor.
   * @param Camera   Photon Camera instance to pull all measurement data from
   * @param Relative Robot-relative position of the camera on the robot
   */
  public VisionCamera(final PhotonCamera Camera, final Transform3d Relative) {
    super(Camera.getCameraTable(), Relative, Camera.getName());
    CAMERA = Camera;
    RELATIVE = Relative;

    POSE_ESTIMATOR = new PhotonPoseEstimator(
      Measurements.FIELD_LAYOUT, 
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
      CAMERA,
      RELATIVE);
    TIMESTAMPS = new ArrayList<>();
    POSES = new ArrayList<>();
  }

  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void periodic(){
    if(CAMERA.isConnected()) {
      update();    
      getRobotPosition().ifPresent((Pose) -> POSE_ESTIMATOR.setLastPose(Pose));
    }
  }

  @Override
  public void update() {
    synchronized(CAMERA) {
      if(CAMERA.isConnected()) {
        Logger.recordOutput(IDENTITY + "/Connected", getConnected());
        Logger.recordOutput(IDENTITY + "/Latency", getLatency());

        getRobotPosition().ifPresent((Pose) -> Logger.recordOutput(IDENTITY + "/RobotPose", Pose));
        Logger.recordOutput(IDENTITY + "/Deltas", getRobotPositionDeltas());
        Logger.recordOutput(IDENTITY + "/Timestamps", getRobotPositionTimestamps());
        
        getOptimalTarget().ifPresent((BestTarget) -> Logger.recordOutput(IDENTITY + "/OptimalTransform", BestTarget));
        getObjectFieldPose().ifPresent((TargetPose) -> Logger.recordOutput(IDENTITY + "/TargetPose", TargetPose));
        Logger.recordOutput(IDENTITY + "/Target" + ']', getTargets().stream().map((Target) -> {
          try {
            return Target.get();
          } catch(final NoSuchElementException Exception) {
            return (null);
          }
        }).toArray(Transform3d[]::new));
        Logger.recordOutput(IDENTITY + "/HasTargets", hasTargets());
        Logger.recordOutput(IDENTITY + "/AmountTarget", getNumTargets());        
      }
    } 
  }

  @Override
  public synchronized void snapshotInput() {
    synchronized(CAMERA) {
      CAMERA.takeInputSnapshot();
    }
  }

  @Override
  public synchronized void snapshotOutput() {
    synchronized(CAMERA) {
      CAMERA.takeOutputSnapshot();
    }
  }

  @Override
  public void close() throws IOException {
    TIMESTAMPS.clear();
    POSES.clear();
    CAMERA.close();
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  @Override
  public Matrix<N3, N1> getStandardDeviations() {
    final var Length = POSES.size();

    final var X_Estimates = new double[Length];
    final var Y_Estimates = new double[Length];
    final var Z_Estimates = new double[Length];  

    for(Integer Index = (0); Index < POSES.size(); Index++){
      final var Estimate = POSES.get(Index);
      X_Estimates[Index] = Estimate.getX();
      Y_Estimates[Index] = Estimate.getY();
      Z_Estimates[Index] = Estimate.getZ();
    }

    return MatBuilder.fill(Nat.N3(), Nat.N1(), new double[] {
      standardDeviation(X_Estimates),
      standardDeviation(Y_Estimates),
      standardDeviation(Z_Estimates)
    });
  }
 
  /**
   * Provides the standard deviation for a given set of numbers 
   * @param Numbers Collection (array) of data to find the standard deviation of
   * @return Standard deviation as a double value
   */
  private double standardDeviation(double[] Numbers){
    Double Mean = 0d, SummativeSquareDifference = 0d;
    for(double Number : Numbers){
      Mean += Number;
    }
    Mean /= Numbers.length;
    for(double Number : Numbers){
      SummativeSquareDifference += Math.pow((Number - Mean), (2));
    }
    return Math.sqrt(SummativeSquareDifference / Numbers.length - 1);
  }

  @Override
  public Optional<Matrix<N3, N3>> getCameraMatrix() {
    return CAMERA.isConnected()? CAMERA.getCameraMatrix(): Optional.empty();
  }

  @Override
  public Optional<Matrix<N5, N1>> getCoefficientMatrix() {
    return CAMERA.isConnected()? CAMERA.getDistCoeffs(): Optional.empty();
  }

  @Override
  public double[] getRobotPositionTimestamps() {
    return TIMESTAMPS.stream().mapToDouble(Double::doubleValue).toArray();
  }

  @Override
  public Pose3d[] getRobotPositionDeltas() {
    final Pose3d[] Deltas = new Pose3d[POSES.size() - (1)];
    for(Integer Index =  (0); Index < POSES.size(); Index++){
      final var Previous = POSES.get(Index);
      final var Current = POSES.get(Index+(1));
      Deltas[Index] = new Pose3d(
        Current.getX() - Previous.getX(),
        Current.getY() - Previous.getY(),
        Current.getZ() - Previous.getZ(),
        new Rotation3d(
          Current.getRotation().getX() - Previous.getRotation().getX(),
          Current.getRotation().getY() - Previous.getRotation().getY(),
          Current.getRotation().getZ() - Previous.getRotation().getZ()
        )
      );
    }
    return Deltas;
  }

  @Override
  public Optional<Pose3d> getRobotPosition() {
    if(CAMERA.isConnected()) {
      final var Estimate = POSE_ESTIMATOR.update().get();
      if(!POSES.isEmpty()) {
        POSE_ESTIMATOR.setLastPose(Estimate.estimatedPose);
      }
      POSES.add(Estimate.estimatedPose);
      TIMESTAMPS.add(Estimate.timestampSeconds);
      return Optional.ofNullable(Estimate.estimatedPose);      
    }
    return Optional.empty();
  }

  @Override
  public Optional<Pose3d> getObjectFieldPose() {
    if(CAMERA.isConnected()) {
      final var RobotEstimate = getRobotPosition();
      final var OptimalEstimate = getOptimalTarget();
      if(RobotEstimate.isEmpty() || OptimalEstimate.isEmpty()){
        return Optional.empty();
      } 
      return Optional.of(new Pose3d(
        (OptimalEstimate.get().getX() + RobotEstimate.get().getX()),
        (OptimalEstimate.get().getY() + RobotEstimate.get().getY()),
        OptimalEstimate.get().getZ(),
        new Rotation3d()
        ));
    }
    return Optional.empty();
  }

  @Override
  public Optional<Pose3d> getObjectFieldPose(final Transform3d Target) {
    if(CAMERA.isConnected()) {
      Optional<Pose3d> RobotEstimate = getRobotPosition();
      if(RobotEstimate.isEmpty()){
        return Optional.empty();
      }
      return Optional.of(new Pose3d(
        (Target.getX() + RobotEstimate.get().getX()),
        (Target.getY() + RobotEstimate.get().getY()),
        Target.getZ(),
        new Rotation3d()
      ));
    }
    return Optional.empty();
  }

  @Override
  public List<Optional<Transform3d>> getTargets() {
    if(CAMERA.isConnected()) {
      final var LatestResult = CAMERA.getLatestResult().getTargets();
      if(LatestResult.isEmpty()){
        return List.of();
      }
      return CAMERA.getLatestResult().getTargets().stream()
        .map(PhotonTrackedTarget::getBestCameraToTarget)
        .map(Optional::ofNullable)
        .toList();
    }
    return List.of();
  }

  @Override
  public boolean hasTargets(){
    return CAMERA.isConnected()? CAMERA.getLatestResult().hasTargets(): (false);
  }

  @Override
  public int getNumTargets(){
    return CAMERA.isConnected()? CAMERA.getLatestResult().getTargets().size(): (0);
  }


  @Override
  public Optional<Transform3d> getOptimalTarget() {
    if(CAMERA.getLatestResult().getBestTarget() == (null)){
      return Optional.empty();
    }
    return Optional.of(CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget());
  }

  @Override
  public Double getLatency(){
    return CAMERA.isConnected()? CAMERA.getLatestResult().getLatencyMillis(): (-1d);
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