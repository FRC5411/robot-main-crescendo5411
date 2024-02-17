// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.util.Queue;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;
import java.util.concurrent.atomic.AtomicReference;

import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;

import com.jcabi.aspects.Timeable;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.concurrent.ArrayBlockingQueue;
// -----------------------------------------------------------[Trajectory Manager]----------------------------------------------------------//
/**
 * 
 *
 * <h1>TrajectoryManager</h1>
 * 
 * <p>Manages a set of Trajectory Solvers to solve for a list of objects
 * 
 * @see Thread
 */
public class TrajectoryManager extends Thread implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectoryManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectoryManager() {
    start();
  } static {

  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Starts the Manager, should not explicitly be called explicitly.
   * @throws IllegalAccessError When this method is called explicitly
   */
  @Override
  public synchronized void start() throws IllegalAccessError {
    super.start();
  }


  /**
   * Operates upon the worker TrajectoryThreads, manages allocations, efficiency, etcetera
   */
  @Override
  public synchronized void run() {
    while(isAlive()) {



      if(isInterrupted()) {

      }
    }
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  @Override
  public synchronized void close() {

  }


  /**
   * Submits a game piece to the trajectory solver
   * @param Object Object to solve for the trajectory of
   */
  public synchronized void submit(final SolvableObject Object) {

  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * <h2>SolvableObject</h2>
   * 
   * Represents an object with the necessary values for a trajectory to be created
   */
  public static class SolvableObject {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    public final InterpolatingDoubleTreeMap TRAJECTORY;
    public final Double FIRING_VELOCITY;
    public final Double FIRING_ROTATION;
    public final Double MASS;
    public final Double MU;    
    public final Double TERMINAL;    

    public static final Double NOTE_MU = (1.17d);
    public static final Double NOTE_MASS = (2.35301e-1d);
    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
    /**
     * Solvable Object Constructor
     * @param Velocity Speed of the object upon entering the trajectory     (m/s)
     * @param Rotation Rotation of the object upon entering the trajectory  (rad)
     * @param Mass     Constant mass of the object                          (kg)
     * @param Mu       Friction coefficient by the air acting on the object (None)
     * @param Horizon  Distance for which this object is a projectile       (m)
     */
    public SolvableObject(final Double Velocity, final Double Rotation, final Double Mass, final Double Mu, final Double Horizon) {
      FIRING_VELOCITY = Velocity;
      FIRING_ROTATION = Rotation;
      MASS = Mass;
      MU = Mu;
      TERMINAL = Horizon;
      TRAJECTORY = new InterpolatingDoubleTreeMap();
    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Quickly creates a note preset of this object
     * @param Velocity Speed of the object upon entering the trajectory     (m/s)
     * @param Rotation Rotation of the object upon entering the trajectory  (rad)
     * @param Horizon  Distance for which this object is a projectile       (m)
     * @return
     */
    public static SolvableObject note(final Double Velocity, final Double Rotation, final Double Horizon) {
      return new SolvableObject(Velocity, Rotation, NOTE_MASS, NOTE_MU, Horizon);
    }

  }

  /**
   *
   * <h1>TrajectorySolverManager</h1>
   * 
   * <p>Solves for the specific values of an object in real space.<p>
   * 
   * @see Thread
   */
  private static class TrajectorySolver implements Closeable, Runnable {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private final Queue<SolvableObject> OBJECT_QUEUE;
    private final Integer OBJECT_QUEUE_MAXIMUM_ELEMENTS = (20);
    private final Boolean OBJECT_QUEUE_ORDERED = (true);
    private final Double ENVIRONMENTAL_ACCELERATION = (-9.80665d);
    private final Double ENVIRONMENTAL_DENSITY = (1.1839d);
    private final Integer HORIZON_MAXIMUM_SAMPLES = (100);
    private final Integer HORIZON_MAXIMUM_PASSTHROUGH = (1000);
    // -------------------------------------------------------------[Fields]----------------------------------------------------------------/][]'
    private volatile Integer PrecedingHorizon = (1);
    private volatile Double PrecedingPosition = (0d);
    private volatile Double HorizonAccuracy = (0d);
    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
    /**
     * Trajectory Solver Constructor.
     */
    public TrajectorySolver() {
      OBJECT_QUEUE = new ArrayBlockingQueue<>(OBJECT_QUEUE_MAXIMUM_ELEMENTS, OBJECT_QUEUE_ORDERED);
    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//

    @Timeable(limit = 200, unit = TimeUnit.MILLISECONDS)
    public synchronized void run() {
      synchronized(OBJECT_QUEUE) {
        while(!Thread.currentThread().isInterrupted() && !OBJECT_QUEUE.isEmpty()) {
          final var Object = OBJECT_QUEUE.peek();
          final var Horizon = Object.TERMINAL > HORIZON_MAXIMUM_SAMPLES? HORIZON_MAXIMUM_SAMPLES: Object.TERMINAL;
          final var HorizontalSurfaceArea =
            (Math.PI * (Measurements.OBJECT_OUTER_RADIUS - Measurements.OBJECT_INNER_RADIUS)) * 
            (Math.abs(Math.cos(Object.FIRING_ROTATION)) + (1)) * Math.PI * Measurements.OBJECT_OUTER_RADIUS;             
          final var VerticalSurfaceArea =
            (Math.PI * (Measurements.OBJECT_OUTER_RADIUS - Measurements.OBJECT_INNER_RADIUS)) * 
            (Math.abs(Math.sin(Object.FIRING_ROTATION)) + (1)) * Math.PI * Measurements.OBJECT_OUTER_RADIUS;   
          var HorizontalVelocity = new AtomicReference<Double>(Object.FIRING_VELOCITY * Math.cos(Object.FIRING_ROTATION));       
          var VerticalVelocity = new AtomicReference<Double>(Object.FIRING_VELOCITY * Math.sin(Object.FIRING_ROTATION));    
          HorizonAccuracy = (double) (Horizon) / HORIZON_MAXIMUM_PASSTHROUGH;
          IntStream.rangeClosed(PrecedingHorizon, (int) Math.floor(Horizon * (1 / HorizonAccuracy))).forEachOrdered((final int HorizonPoint) -> {
            final var HorizontalPosition = PrecedingPosition + (HorizontalVelocity.get() * HorizonAccuracy);
            final var VerticalPosition = Object.TRAJECTORY.get(PrecedingPosition) + (HorizontalVelocity.get() * HorizonAccuracy); 
            PrecedingPosition = HorizontalPosition;    
            HorizontalVelocity.set(HorizontalVelocity.get() - (
            (drag(
              Math.cos(Object.FIRING_ROTATION) * HorizontalVelocity.get(),
              HorizontalSurfaceArea,
              Object.MU
            ) / Object.MASS)) * HorizonAccuracy);
            VerticalVelocity.set(VerticalVelocity.get() - (ENVIRONMENTAL_ACCELERATION + 
            (drag(
              Math.sin(Object.FIRING_ROTATION) * VerticalVelocity.get(),
              VerticalSurfaceArea,
              Object.MU
            ) / Object.MASS)) * HorizonAccuracy);
            Object.TRAJECTORY.put(HorizontalPosition, VerticalPosition);
            PrecedingHorizon = HorizonPoint;
          });
        }
      }   
    }

    /**
     * Closes this instance and all held resources immediately.
     */
    @Override
    public synchronized void close() {
      OBJECT_QUEUE.clear();
    }

    /**
     * Calculates the force of drag on a real world object according to the equation:
     * <pre><code>
     * F = 1/2 * p * v^2 * a * c 
     * </code></pre>
     * With the density of air 1.1839                            (Kg/m^3)
     * @param Velocity Relative velocity of the object in motion (m/s)
     * @param Area     Cross-sectional area of a given object    (m^2)
     * @param Mu       Friction coefficient of the object        (None)
     * @return The force, in Newtons of the object               (N)
     */
    public Double drag(final Double Velocity, final Double Area, final Double Mu) {
      return (1/2) * ENVIRONMENTAL_DENSITY * Math.pow(Velocity,(2)) * Mu * Area;
    }
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
    /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized TrajectoryManager getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new TrajectoryManager();
      return Instance;
  }

}
