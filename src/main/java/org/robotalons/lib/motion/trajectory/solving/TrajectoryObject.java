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
  public double MU;
  public double MASS;  
  public double HORIZON;
  public double VERTICAL;
  public int ITERATIONS;
  public double VERTICAL_AREA;
  public double HORIZONTAL_AREA;
  public double OFFSET_LENGTH;    
  public double INITIAL_VELOCITY;
  public Rotation2d INITIAL_ROTATION;
  public Rotation2d OPTIMIZED_ROTATION;
  
  public static final double NOTE_MU = (1.17d);
  public static final double NOTE_MASS = (2.35301e-1d);
  public static final double NOTE_INNER_RADIUS = Units.inchesToMeters((10d));
  public static final double NOTE_OUTER_RADIUS = Units.inchesToMeters((14d));

  /**
   * Constructs a note preset of a Trajectory Object
   * @param Velocity  Initial Velocity
   * @param Rotation  Initial Rotation
   * @param Offset    Length of the offset's slope (shooter length)
   * @param Distance  How far lengthwise the object must travel
   * @param Height    How far heightwise the object must travel
   * @param Iterations How many discrete points to solve for along the trajectory
   * @return Note preset with the parameters
   */
  public static final TrajectoryObject note(
      final double Velocity, final Rotation2d Rotation, final double Offset,
      final double Distance, final double Height, final int Iterations) {
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
