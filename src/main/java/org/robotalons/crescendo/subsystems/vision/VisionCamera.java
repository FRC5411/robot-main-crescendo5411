// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
      //TODO: Update This
      //Logger.processInputs(IDENTITY, CAMERA_STATUS);
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
    final var X_Estimates = new double[POSES.size()];
    final var Y_Estimates = new double[POSES.size()];
    final var Z_Estimates = new double[POSES.size()];  

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
  private static double standardDeviation(double[] Numbers){
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
  public Pose2d[] getRobotPositionDeltas() {
    final Pose2d[] Deltas = new Pose2d[POSES.size()];
    for(Integer Index = (0); Index < POSES.size() - (1); Index++){
      final var Previous = POSES.get(Index);
      final var Current = POSES.get(Index + (1));
      Deltas[Index] = new Pose2d(
        Current.getX() - Previous.getX(),
        Current.getY() - Previous.getY(),
        new Rotation2d(
          Current.getRotation().getX() - Previous.getRotation().getX(),
          Current.getRotation().getY() - Previous.getRotation().getY()
        )
      );
    }

    return Deltas;
  }

  @Override
  public Optional<Pose3d> getRobotPosition() {
    if(CAMERA.isConnected()) {
      final var Estimate = POSE_ESTIMATOR.update();
      if(Estimate.isPresent()) {
        if(!POSES.isEmpty()) {
          POSE_ESTIMATOR.setLastPose(Estimate.get().estimatedPose);
        }
        POSES.add(Estimate.get().estimatedPose);
        TIMESTAMPS.add(Estimate.get().timestampSeconds);
        return Optional.ofNullable(Estimate.get().estimatedPose);          
      }
    }
    return Optional.empty();
  }

  // TODO: Check to make sure that ur good with these being pose2d
  @Override
  public Optional<Pose2d> getObjectFieldPose() {
    if(CAMERA.isConnected()) {
      final var RobotEstimate = getRobotPosition();
      final var OptimalEstimate = getOptimalTarget();
      
      if(RobotEstimate.isEmpty() || OptimalEstimate.isEmpty()){
        return Optional.empty();
      } 
      return Optional.ofNullable(new Pose2d(
        (OptimalEstimate.get().getX() + RobotEstimate.get().getX()),
        (OptimalEstimate.get().getY() + RobotEstimate.get().getY()),
        new Rotation2d()
      ));
    }
    return Optional.empty();
  }

  @Override
  public Optional<Pose2d> getObjectFieldPose(final Transform2d Target) {
    if(CAMERA.isConnected()) {
      final var RobotEstimate = getRobotPosition();
      
      if(RobotEstimate.isEmpty()){
        return Optional.empty();
      } 
      return Optional.ofNullable(new Pose2d(
        (Target.getX() + RobotEstimate.get().getX()),
        (Target.getY() + RobotEstimate.get().getY()),
        new Rotation2d()
      ));
    }
    return Optional.empty();
  }

  //TODO: Check Cody to make sure ur fine with this
  @Override
  public List<Optional<Transform2d>> getTargets() {
    if(CAMERA.isConnected()) {
      final var LatestResult = CAMERA.getLatestResult().getTargets();
      if(LatestResult.isEmpty()){
        return List.of();
      }

      List<Optional<Transform3d>> Targets3D = CAMERA.getLatestResult().getTargets().stream()
      .map(PhotonTrackedTarget::getBestCameraToTarget)
      .map(Optional::ofNullable)
      .toList();

      List<Optional<Transform2d>> Targets2D = new ArrayList<>();

      for(int i = 0; i < Targets3D.size(); i++){
        Double x = Targets3D.get(i).get().getX();
        Double y = Targets3D.get(i).get().getY();
        
        Targets2D.set(i, Optional.of(new Transform2d(x, y, new Rotation2d())));
      }

      return Targets2D;
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

  //TODO Check pls Cody
  @Override
  public Optional<Transform2d> getOptimalTarget() {
    if(CAMERA.getLatestResult().getBestTarget() == (null) ){
      return Optional.empty();
    }

    Transform3d target3D = CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget();

    return Optional.ofNullable(new Transform2d(target3D.getX(), target3D.getY(), new Rotation2d()));
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