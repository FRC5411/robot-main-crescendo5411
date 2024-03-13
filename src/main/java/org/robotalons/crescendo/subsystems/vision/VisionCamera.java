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
import org.robotalons.lib.utilities.MathUtilities;
import org.robotalons.lib.vision.Camera;

import java.util.Objects;
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
  private final PhotonPoseEstimator POSE_ESTIMATOR;
  private final PhotonCamera CAMERA;
  private final Transform3d RELATIVE;  
  private final List<Double> TIMESTAMPS;
  private final List<Pose3d> POSES;
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
  public synchronized void periodic(){
    if(CAMERA.isConnected()) {
      update();    
      getRobotPosition().ifPresent((Pose) -> POSE_ESTIMATOR.setLastPose(Pose));
    }
  }

  @Override
  public synchronized void update() {
    synchronized(CAMERA) {
      if(CAMERA.isConnected()) {
        synchronized(STATUS) {
          STATUS.Connected = CAMERA.isConnected();
          STATUS.Deltas = getRobotPositionDeltas();
          STATUS.Robot = POSE_ESTIMATOR.getReferencePose().toPose2d();
          STATUS.Timestamps = getRobotPositionTimestamps();
          STATUS.Latency = CAMERA.getLatestResult().getLatencyMillis();
          Logger.processInputs(IDENTITY + "/Camera", STATUS);
          getOptimalTarget().ifPresent((Target) -> {
            STATUS.OptimalTargetTransform = Target;
            STATUS.OptimalTargetPose = getObjectFieldPose(Target).get();
          });
          STATUS.RealizedTargets = getTargets().stream().toArray(Transform2d[]::new);
          Logger.processInputs(IDENTITY + "/Target", STATUS);
        }
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
      MathUtilities.standardDeviation(X_Estimates),
      MathUtilities.standardDeviation(Y_Estimates),
      MathUtilities.standardDeviation(Z_Estimates)
    });
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

  @Override
  public List<Transform2d> getTargets() {
    if(CAMERA.isConnected()) {
      final var LatestResult = CAMERA.getLatestResult().getTargets();
      if(LatestResult.isEmpty()){
        return List.of();
      }
      return CAMERA.getLatestResult().getTargets().stream()
        .map(PhotonTrackedTarget::getBestCameraToTarget)
        .filter(Objects::isNull)
        .map((Target) -> scope(new Transform2d(Target.getTranslation().toTranslation2d(), Target.getRotation().toRotation2d())))
        .toList();
    }
    return List.of();
  }
  
  /**
   * Changes the scope of this Transformation to be robot-relative as opposed to it's original, camera-relative form.
   * @param Transformation Original Transformation, camera-relative
   * @return Robot-relative transformation, if the input was not null
   */
  private Transform2d scope(final Transform2d Transformation) {
    try {
      final var Imprint = RELATIVE.times((-1));
      return Transformation.plus(new Transform2d(Imprint.getTranslation().toTranslation2d(), Imprint.getRotation().toRotation2d()));      
    } catch(final NullPointerException Ignored) {
      return (null);
    }
  }

  @Override
  public Optional<Transform2d> getOptimalTarget() {
    if(CAMERA.getLatestResult().getBestTarget() == (null) ){
      return Optional.empty();
    }
    Transform3d Target = CAMERA.getLatestResult().getBestTarget().getBestCameraToTarget();
    return Optional.ofNullable(scope(new Transform2d(Target.getTranslation().toTranslation2d(), Target.getRotation().toRotation2d())));
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