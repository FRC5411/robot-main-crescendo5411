// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory.solving;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

// -----------------------------------------------------------[Trajectory Object]-----------------------------------------------------------//
/**
 * <h1>TrajectoryObject</h1>
 * 
 * Represents a container for an object with a trajectory that has values which are unknown and need to be
 * solved for.
 */
public class TrajectoryObject {
  public Double MU;
  public Double MASS;  
  public Double HORIZON;
  public Double VERTICAL;
  public Integer ITERATIONS;
  public Double VERTICAL_AREA;
  public Double HORIZONTAL_AREA;
  public Double OFFSET_LENGTH;    
  public Double INITIAL_VELOCITY;
  public Rotation2d INITIAL_ROTATION;
  public Rotation2d OPTIMIZED_ROTATION;
}
