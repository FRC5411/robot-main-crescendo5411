// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory.solving;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
  
  public static final Double NOTE_MU = (1.17d);
  public static final Double NOTE_MASS = (2.35301e-1d);
  public static final Double NOTE_INNER_RADIUS = Units.inchesToMeters((10d));
  public static final Double NOTE_OUTER_RADIUS = Units.inchesToMeters((14d));

  /**
   * Constructs a note preset of a Trajectory Object
   * @param Velocity  Initial Velocity
   * @param Rotation  Initial Rotation
   * @param Offset    Length of the offset's slope (shooter length)
   * @param Distance  How far lengthwise the object must travel
   * @param Height    How far height-wise the object must travel
   * @param Iterations How many discrete points to solve for along the trajectory
   * @return Note preset with the parameters
   */
  public static TrajectoryObject note(
    final Double Velocity, final Rotation2d Rotation, final Double Offset,
    final Double Distance, final Double Height, final Integer Iterations) {
    var Object = new TrajectoryObject();
    Object.MU = NOTE_MU;
    Object.MASS = NOTE_MASS;
    Object.HORIZON = Distance;
    Object.VERTICAL = Height;
    Object.ITERATIONS = Iterations;
    Object.OFFSET_LENGTH = Offset;
    Object.INITIAL_ROTATION = Rotation;
    Object.INITIAL_VELOCITY = Velocity;
    Object.HORIZONTAL_AREA = (Math.PI * (NOTE_OUTER_RADIUS - NOTE_INNER_RADIUS)) * 
      (Math.abs(Math.cos(Rotation.getRadians())) + (1)) * Math.PI * NOTE_OUTER_RADIUS;
    Object.VERTICAL_AREA = (Math.PI * (NOTE_OUTER_RADIUS - NOTE_INNER_RADIUS)) * 
      (Math.abs(Math.sin(Rotation.getRadians())) + (1)) * Math.PI * NOTE_OUTER_RADIUS;
    return Object;
  }
}
