// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.pathfinding.Pathfinding;

import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem.CameraIdentifier;
import org.robotalons.lib.TalonSubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
// -----------------------------------------------------------[Subsystem Manager]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Subsystem Manager</h1>
 *
 * <p>Contains implementation for subsystems interfacing between each other to receive serialized information./p>
 *
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public final class SubsystemManager extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final List<TalonSubsystemBase> SUBSYSTEMS;
  public static final TalonSubsystemBase VISION;
  public static final Field2d FIELD;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static SubsystemManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private SubsystemManager() {} static {
    SUBSYSTEMS = new ArrayList<>();
    FIELD = new Field2d();
    VISION = VisionSubsystem.getInstance();
    //TODO: <... Fetch Other Subsystem Instances>
    SUBSYSTEMS.add(VISION);
    //TODO: <... Add Other Subsystem Instances>
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    Pathfinding.ensureInitialized();  
  }

  /**
   * Ensures that the pathfinder is initialized, and that the start position of path finding is correct.
   */
  public static void ensure() {
    Pathfinding.ensureInitialized();
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the transform in three-dimensional space to the optimal target of a given camera
   * @param Port Which port to pull data from, ranging from 0, up to 4
   * @return Transformation in three-dimensional space camera relative
   */
  public static Optional<Transform3d> getOptimalTarget(final CameraIdentifier Port) {
    return VisionSubsystem.getOptimalTarget(Port);
  }

  /**
   * Provides the employee subsystems managed my this subsystem manager
   * @return List of dependent subsystems
   */
  public static List<TalonSubsystemBase> getSubsystems() {
    return SUBSYSTEMS;
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized SubsystemManager getInstance() {
      if (java.util.Objects.isNull(Instance)) {
          Instance = new SubsystemManager();
      }
      return Instance;
  }
}
