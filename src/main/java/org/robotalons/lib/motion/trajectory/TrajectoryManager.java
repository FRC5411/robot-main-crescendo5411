// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.util.Queue;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;
import java.util.concurrent.atomic.AtomicReference;
import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Callable;
import java.util.Optional;

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
  public static final Integer MAXIMUM_WORKERS = (10);
  private static final List<SolvableObject> INPUT_CACHE;  
  private static final List<SolvableObject> OUTPUT_CACHE;
  private static final List<TrajectorySolver> WORKERS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectoryManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectoryManager() {
    start();
  } static {
    INPUT_CACHE = new ArrayList<>();
    OUTPUT_CACHE = new ArrayList<>();
    WORKERS = new ArrayList<>();
    IntStream.rangeClosed((0), MAXIMUM_WORKERS).forEachOrdered((final int Worker) -> {
      WORKERS.add(new TrajectorySolver());
    });


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
      synchronized(INPUT_CACHE) {
        while(!INPUT_CACHE.isEmpty()) {
          if(WORKERS.stream().mapToInt((Worker) -> Worker.OBJECT_QUEUE.size()).sum() == TrajectorySolver.OBJECT_QUEUE_MAXIMUM_ELEMENTS * WORKERS.size()) {
            WORKERS.add(new TrajectorySolver());
          }
          final var Map = new HashMap<TrajectorySolver,Integer>();
          WORKERS.forEach((Worker) -> Map.put(Worker, Worker.OBJECT_QUEUE.size()));
          final var Iterator = Map.keySet().iterator();
          INPUT_CACHE.forEach((Object) -> {
            final var Solver = Iterator.next();
            Map.values().stream().mapToInt((Value) -> Value).min().ifPresentOrElse(
            (Minimum) -> {
              if(Solver.OBJECT_QUEUE.size() == Minimum) {
                Solver.OBJECT_QUEUE.offer(Object);
                Map.replace(Solver, Solver.OBJECT_QUEUE.size());
              }
            },
            () -> {
              Solver.OBJECT_QUEUE.offer(Object);
            });
          });
        }
        WORKERS.parallelStream().forEach((Worker) -> {
          Worker.call().ifPresent(
          (Object) -> {
            OUTPUT_CACHE.add(Object);
          });
        });
      }
    }
  }

  /**
   * Closes this instance and all held resources.
   */
  @Override
  public synchronized void close() {
    while(WORKERS.stream().mapToInt((Worker) -> Worker.OBJECT_QUEUE.size()).sum() > (0)) {
      WORKERS.parallelStream().forEach((Worker) -> {
        Worker.call().ifPresent(
          (Object) -> {
            OUTPUT_CACHE.add(Object);
          });
        if(Worker.OBJECT_QUEUE.isEmpty()) {
          Worker.close();
        }
      });
    }
    WORKERS.clear();

  }


  /**
   * Submits a Solvable Object to the trajectory solver, which will be added to a thread for evaluation during the next control loop
   * @param Object Object to solve for the trajectory of
   */
  public synchronized CompletableFuture<SolvableObject> submit(final SolvableObject Object) {
    INPUT_CACHE.add(Object);
    return CompletableFuture.supplyAsync(() -> {
      return OUTPUT_CACHE.stream().filter((Solved) -> {
        return Solved.equals(Object);
      }).toList().get((0));
    });
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
  private static class TrajectorySolver implements Closeable, Callable<Optional<SolvableObject>> {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    public final Queue<SolvableObject> OBJECT_QUEUE;
    public final static Integer OBJECT_QUEUE_MAXIMUM_ELEMENTS = (20);
    private final static Boolean OBJECT_QUEUE_ORDERED = (true);
    private final static Double ENVIRONMENTAL_ACCELERATION = (-9.80665d);
    private final static Double ENVIRONMENTAL_DENSITY = (1.1839d);
    private final static Integer HORIZON_MAXIMUM_SAMPLES = (100);
    private final static Integer HORIZON_MAXIMUM_PASSTHROUGH = (1000);
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
    public synchronized Optional<SolvableObject> call() {
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
          OBJECT_QUEUE.poll();
          return Optional.of(Object);
        }
        return Optional.empty();
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
    private static Double drag(final Double Velocity, final Double Area, final Double Mu) {
      return (1/2) * ENVIRONMENTAL_DENSITY * Math.pow(Velocity,(2)) * Mu * Area;
    }
  }
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
