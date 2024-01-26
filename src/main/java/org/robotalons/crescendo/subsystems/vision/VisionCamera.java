// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robotalons.lib.vision.Camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
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
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  public VisionCamera(final PhotonCamera Camera, final Transform3d Relative, final String Identifier) {
    super(Camera.getCameraTable(), Relative, Identifier);
    CAMERA = Camera;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public void update() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'update'");
  }

  @Override
  public void snapshot() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'snapshot'");
  }

  @Override
  public void close() throws IOException {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }

  @Override
  public void set(VisionLEDMode Mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public void set(Integer Mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public Matrix<Num, N1> getStandardDeviations() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getStandardDeviations'");
  }

  @Override
  public Optional<Matrix<N3, N3>> getCameraMatrix() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCameraMatrix'");
  }

  @Override
  public Optional<Matrix<N5, N1>> getCoefficientMatrix() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCoefficientMatrix'");
  }

  @Override
  public Pair<Integer, Integer> getImageResolution() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getImageResolution'");
  }

  @Override
  public List<Double> getRobotPositionTimestamps() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotPositionTimestamps'");
  }

  @Override
  public List<Pose3d> getRobotPositionDeltas() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotPositionDeltas'");
  }

  @Override
  public Pose3d getRobotPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotPosition'");
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
