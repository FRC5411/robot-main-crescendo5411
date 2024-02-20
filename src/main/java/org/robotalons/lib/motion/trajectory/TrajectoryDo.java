// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.util.Queue;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;
import java.util.concurrent.atomic.AtomicReference;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Callable;
import java.util.Optional;

import com.jcabi.aspects.Timeable;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import java.util.concurrent.ArrayBlockingQueue;
// -----------------------------------------------------------[Trajectory Manager]----------------------------------------------------------//
/**
 * 
 *
 * <h1>TrajectoryManager</h1>
 * 
 * <p>Manages a set of Trajectory Solvers to solves for the complete trajectories upon a provided target, or the desired
 * firing rotation or velocity given rotation or velocity.
 * 
 * @see Thread
 */
public class TrajectoryDo extends Thread implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Integer MAXIMUM_WORKERS = (10);
  private static final List<SolvableObject> INPUT_CACHE;  
  private static final List<SolvableObject> OUTPUT_CACHE;
  private static final List<TrajectorySolver> WORKERS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectoryDo Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectoryDo() {
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
   */
  @Override
  public synchronized void start() {
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
  public static synchronized CompletableFuture<SolvableObject> submit(final SolvableObject Object) {
    //TODO: Check for repeats

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
    public final Optional<Double> FIRING_VELOCITY;
    public final Optional<Double> FIRING_ROTATION;
    public final Double MASS;
    public final Double MU;    
    public final Double TERMINAL; 
    public final Double HORIZONTAL_AREA;
    public final Double VERTICAL_AREA;
    public final Double INITIAL_POSITION;

    public static final Double NOTE_MU = (1.17d);
    public static final Double NOTE_MASS = (2.35301e-1d);
    public static final Double NOTE_INNER_RADIUS = Units.inchesToMeters((10d));
    public static final Double NOTE_OUTER_RADIUS = Units.inchesToMeters((14d));
    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
    /**
     * Solvable Object Constructor
     * @param Velocity   Speed of the object upon entering the trajectory         (m/s)
     * @param Rotation   Rotation of the object upon entering the trajectory      (rad)
     * @param Mass       Constant mass of the object                              (kg)
     * @param Mu         Friction coefficient by the air acting on the object     (None)
     * @param Horizon    Distance for which this object is a projectile           (m)
     * @param Vertical   Initial vertical position upon entering the trajectory   (m)
     * @param Horizontal Initial horizontal position upon entering the trajectory (m)
     */
    public SolvableObject(
       final Double Velocity, final Double Rotation, final Double Mass, final Double Mu, final Double Horizon,
       final Double HorizontalPosition, final Double VerticalPosition, final Double HorizontalArea, final Double VerticalArea) {
      FIRING_VELOCITY = Optional.ofNullable(Velocity);
      FIRING_ROTATION = Optional.ofNullable(Rotation);
      MASS = Mass;
      MU = Mu;
      TERMINAL = Horizon;
      HORIZONTAL_AREA = HorizontalArea;
      VERTICAL_AREA = VerticalArea;
      TRAJECTORY = new InterpolatingDoubleTreeMap();
      TRAJECTORY.put(HorizontalPosition, VerticalPosition);
      INITIAL_POSITION = HorizontalPosition;
    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Quickly creates a note preset of this object
     * @param Velocity   Speed of the object upon entering the trajectory                 (m/s)
     * @param Rotation   Rotation of the object upon entering the trajectory              (rad)
     * @param Horizon    Distance for which this object is a projectile                   (m)
     * @param HorizontalPosition Initial horizontal position upon entering the trajectory (m)
     * @param VerticalPosition   Initial vertical position upon entering the trajectory   (m)
     * 
     * @return Solvable Object with note properties
     */
    public static SolvableObject note(final Double Velocity, final Double Rotation, final Double Horizon, final Double HorizontalPosition, final Double VerticalPosition) { 
      return new SolvableObject(Velocity, Rotation, NOTE_MASS, NOTE_MU, Horizon, HorizontalPosition, VerticalPosition, (Math.PI * (NOTE_OUTER_RADIUS - NOTE_INNER_RADIUS)) * 
      (Math.abs(Math.cos(Rotation)) + (1)) * Math.PI * NOTE_OUTER_RADIUS, (Math.PI * (NOTE_OUTER_RADIUS - NOTE_INNER_RADIUS)) * 
      (Math.abs(Math.sin(Rotation)) + (1)) * Math.PI * NOTE_OUTER_RADIUS);
    }
  }

  /**
   *
   * <h1>RotationSolver</h1>
   * 
   * <p>Solves for the specific values of a Rotation of a object in real space.<p>
   * 
   * @see Callable
   */
  private static class RotationSolver implements Closeable, Callable<Optional<Double>> {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    public final Queue<SolvableObject> OBJECT_QUEUE;
    public final static Integer OBJECT_QUEUE_MAXIMUM_ELEMENTS = (20);
    private final static Boolean OBJECT_QUEUE_ORDERED = (true);
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
    public RotationSolver() {
      OBJECT_QUEUE = new ArrayBlockingQueue<>(OBJECT_QUEUE_MAXIMUM_ELEMENTS, OBJECT_QUEUE_ORDERED);
    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    @Timeable(limit = 200, unit = TimeUnit.MILLISECONDS)
    public synchronized Optional<Double> call() {
      try{
        synchronized(OBJECT_QUEUE) {
          while(!Thread.currentThread().isInterrupted() && !OBJECT_QUEUE.isEmpty()) { 
            final var Object = OBJECT_QUEUE.peek();
            final var Horizon = Object.TERMINAL > HORIZON_MAXIMUM_SAMPLES? HORIZON_MAXIMUM_SAMPLES: Object.TERMINAL;
            var Rotation = (0d);
            HorizonAccuracy = (double) (Horizon) / HORIZON_MAXIMUM_PASSTHROUGH;
            PrecedingPosition = Object.INITIAL_POSITION;
            for (; PrecedingHorizon <= (int) Math.floor(Horizon * (1 / HorizonAccuracy)); PrecedingHorizon++) {

            };
            OBJECT_QUEUE.poll();
            return Optional.of((0d));
          }
          return Optional.empty();
        }        
      } catch (final NoSuchElementException Exception) {
        return Optional.of((0d));
      }
    }

    /**
     * Closes this instance and all held resources immediately.
     */
    @Override
    public synchronized void close() {
      OBJECT_QUEUE.clear();
    }
  }
  

  /**
   *
   * <h1>TrajectorySolverManager</h1>
   * 
   * <p>Solves for the specific values of a Trajectory of a object in real space.<p>
   * 
   * @see Callable
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
    /**
     * Computes for the trajectory of an object, returns an empty optional if unable to be completed.
     */
    @Timeable(limit = 200, unit = TimeUnit.MILLISECONDS)
    public synchronized Optional<SolvableObject> call() {
      try{
        synchronized(OBJECT_QUEUE) {
          while(!Thread.currentThread().isInterrupted() && !OBJECT_QUEUE.isEmpty()) { 
            final var Object = OBJECT_QUEUE.peek();
            final var Horizon = Object.TERMINAL > HORIZON_MAXIMUM_SAMPLES? HORIZON_MAXIMUM_SAMPLES: Object.TERMINAL;
            var HorizontalVelocity = new AtomicReference<Double>(Object.FIRING_VELOCITY.get() * Math.cos(Object.FIRING_ROTATION.get()));       
            var VerticalVelocity = new AtomicReference<Double>(Object.FIRING_VELOCITY.get() * Math.sin(Object.FIRING_ROTATION.get()));    
            HorizonAccuracy = (double) (Horizon) / HORIZON_MAXIMUM_PASSTHROUGH;
            PrecedingPosition = Object.INITIAL_POSITION;
            for (; PrecedingHorizon <= (int) Math.floor(Horizon * (1 / HorizonAccuracy)); PrecedingHorizon++) {
                final var HorizontalPosition = PrecedingPosition + (HorizontalVelocity.get() * HorizonAccuracy);
                final var VerticalPosition = Object.TRAJECTORY.get(PrecedingPosition) + (HorizontalVelocity.get() * HorizonAccuracy); 
                PrecedingPosition = HorizontalPosition;    
                HorizontalVelocity.set(HorizontalVelocity.get() - (
                (drag(
                  Math.cos(Object.FIRING_ROTATION.get()) * HorizontalVelocity.get(),
                  Object.HORIZONTAL_AREA,
                  Object.MU
                ) / Object.MASS)) * HorizonAccuracy);
                VerticalVelocity.set(VerticalVelocity.get() - (ENVIRONMENTAL_ACCELERATION + 
                (drag(
                  Math.sin(Object.FIRING_ROTATION.get()) * VerticalVelocity.get(),
                  Object.VERTICAL_AREA,
                  Object.MU
                ) / Object.MASS)) * HorizonAccuracy);
                Object.TRAJECTORY.put(HorizontalPosition, VerticalPosition);
              };
              OBJECT_QUEUE.poll();
              return Optional.of(Object);
            }
          return Optional.empty();
        }        
      } catch (final NoSuchElementException Exception) {
        return Optional.of(OBJECT_QUEUE.poll());
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
  public static synchronized TrajectoryDo getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new TrajectoryDo();
      return Instance;
  }

}