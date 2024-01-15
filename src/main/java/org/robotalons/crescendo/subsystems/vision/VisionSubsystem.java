// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.lib.vision.Camera;

import java.io.Closeable;
import java.util.List;
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
  public static final List<Camera> CAMERAS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static VisionSubsystem Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Vision Subsystem Constructor.
   */
  private VisionSubsystem() {} static {
    CAMERAS = List.of(

    );
  }
  
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  public synchronized void close() {

  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized VisionSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance)) {
        Instance = new VisionSubsystem();
      }
      return Instance;
  }
}
