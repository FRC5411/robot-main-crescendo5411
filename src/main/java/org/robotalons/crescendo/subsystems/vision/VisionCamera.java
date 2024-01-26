// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import org.robotalons.lib.vision.Camera;

import java.io.IOException;
// ---------------------------------------------------------[Photon Vision Module]--------------------------------------------------------//
/**
 *
 *
 * <h1>PhotonVisionModule</h1>
 *
 * <p>Implementation of a single Photon Vision Camera Module, which gathers information on it's position and reports back to the subsystem.</p>
 * 
 * @see Module
 * @see DrivebaseSubsystem
 */
public class VisionCamera extends Camera {

  public VisionCamera(String CameraName) {
    super(CameraName);
    //TODO Auto-generated constructor stub to stop error of no constructor 
  }

  @Override
  public void update() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'update'");
  }

  @Override
  public void close() throws IOException {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'close'");
  }
  
}
