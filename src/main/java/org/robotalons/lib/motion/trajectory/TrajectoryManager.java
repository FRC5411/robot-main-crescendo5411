// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import javax.management.InstanceAlreadyExistsException;

import com.jcabi.aspects.Timeable;

import edu.wpi.first.math.geometry.Rotation2d;
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
public class TrajectoryManager extends Thread implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<Solver<?>> SOLVABLE_SOLVERS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectoryManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectoryManager() {
    start();
  } static {
    SOLVABLE_SOLVERS = new ArrayList<>();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public synchronized void run() {

  }

  public synchronized void close() {

  }

  public synchronized void start() {
    super.start();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  public static class NewtonRaphsonSolver extends Solver<Rotation2d> {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private final static Double ENVIRONMENTAL_ACCELERATION = (9.80665d); //TODO: May need a negative?
    // -------------------------------------------------------------[Fields]----------------------------------------------------------------//
    private volatile Double VelocityConstant;
    private volatile Double HorizonConstant;
    private volatile Rotation2d Rotation;
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    public synchronized Optional<Rotation2d> solve(final Double Horizon) {
      if(Math.abs(rotationalTrajectory(Horizon, Reserved.INITIAL_POSITION, Rotation)) > 1e-4) {
        Rotation = Rotation2d.fromRadians(rootTangent(
          Rotation.getRadians(),
          rotationalTrajectory(
              Reserved.HORIZON, Reserved.INITIAL_VELOCITY, Rotation),
          rotationalDiscreteDerivative(
            Reserved.HORIZON, Reserved.INITIAL_VELOCITY, Rotation, Rotation2d.fromRadians(1e-4))));
      } else {
        return Optional.of(Rotation);
      }
      return Optional.empty();
    }

    /**
     * Calculates the for a point along the object's trajectory
     * @param Horizon  Current point along the trajectory
     * @param Velocity Current velocity along the trajectory
     * @param Rotation Current rotation along the trajectory
     * @return Next value of trajectory
     */
    public synchronized Double rotationalTrajectory(final Double Horizon, final Double Velocity, final Rotation2d Rotation) {
      setHorizon(Horizon, Velocity, Rotation);
      return +HorizonConstant * Math.tan(Rotation.getRadians())
        - Math.pow(HorizonConstant, (2)) * VelocityConstant * secantSquared(Rotation)
        + Reserved.INITIAL_POSITION * Math.sin(Rotation.getRadians())
        + (Math.tan(Reserved.INITIAL_ROTATION.getRadians()) * Reserved.INITIAL_POSITION) * Math.sin(Reserved.INITIAL_ROTATION.getRadians());
    }

    /**
     * Calculates the discrete time derivative of the slope of a given point along the trajectory
     * @param Horizon       Current point along the trajectory
     * @param Velocity      Current velocity along the trajectory
     * @param Rotation      Current rotation along the trajectory
     * @param DeltaRotation Current change in rotation in the trajectory
     * @return Time-discrete derivative of the rotation
     */
    public synchronized Double rotationalDiscreteDerivative(final Double Horizon, final Double Velocity, final Rotation2d Rotation, final Rotation2d DeltaRotation) {
      return ( rotationalTrajectory(Reserved.HORIZON, Reserved.INITIAL_VELOCITY, Rotation.plus(DeltaRotation))
              - rotationalTrajectory(Reserved.HORIZON, Reserved.INITIAL_VELOCITY, Rotation))
          / DeltaRotation.getRadians();
    }

    /**
     * Calculates the derivative (instantaneous rate of change)
     * @param Horizon  Current point along the trajectory
     * @param Velocity Current velocity along the trajectory
     * @param Rotation Current rotation along the trajectory
     * @return Value of the derivative at this point
     */
    private synchronized Double rotationalDerivative(final Double Horizon, final Double Velocity, final Rotation2d Rotation) {
      setHorizon(Horizon, Velocity, Rotation);
      return (Math.PI / (180))
      * (HorizonConstant * secantSquared(Rotation)
          - (2)
              * Math.pow(HorizonConstant, (2))
              * VelocityConstant
              * secantSquared(Rotation)
              * Math.tan(Rotation.getRadians())
          - Reserved.INITIAL_POSITION * Math.cos(Math.toDegrees(Rotation.getRadians())));
    }

    /**
     * Provides the tangent line of a given point at the roots of a function
     * @param X     Value of point along the x axis
     * @param Y     Value of point along the y axis
     * @param Slope Slope of this point
     * @return Tangent line at this point
     */
    private static Double rootTangent(final Double X, final Double Y, final Double Slope) {
      return -(Y / Slope) + X;
    }

    /**
     * Provides the value of secant at a given point along the rotation
     * @param Rotation Current rotation along the trajectory
     */
    private static Double secantSquared(final Rotation2d Rotation) {
      return Math.pow((1) / Math.cos(Rotation.getRadians()),( 2));
    }

    /**
     * Mutates the current horizon constant given the current states of this object at a position.
     * @param Horizon  Current point along the trajectory
     * @param Velocity Current velocity along the trajectory
     * @param Rotation Current rotation along the trajectory
     */
    private synchronized void setHorizon(final Double Horizon, final Double Velocity, final Rotation2d Rotation) {
      HorizonConstant =  -(Reserved.HORIZON
            - Horizon
            + Reserved.INITIAL_POSITION * Math.cos(Rotation.getRadians()));
      VelocityConstant = ENVIRONMENTAL_ACCELERATION / (2 * Velocity * Velocity);
    }
  }


  /**
   * 
   * 
   * <h1>Solver</h1>
   * 
   * <p>Represents a solver which solves upon a solvable instance for some type output.</p>
   * 
   * @see Callable
   * @see Optional
   */
  public static abstract class Solver<SolvableType> implements Callable<Optional<SolvableType>>, Closeable {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private static final Integer SOLVING_QUEUE_MAXIMUM_ELEMENTS = (20);
    private static final Boolean SOLVING_QUEUE_IS_ORDERED = (true);
    private static final Integer SOLVING_MAXIMUM_INSTANCES = (10);
    private static final Queue<Solvable<?>> SOLVING_QUEUE = new ArrayBlockingQueue<>(SOLVING_QUEUE_MAXIMUM_ELEMENTS, SOLVING_QUEUE_IS_ORDERED);
    // -------------------------------------------------------------[Fields]----------------------------------------------------------------//
    protected volatile Solvable<SolvableType> Reserved = (null);
    protected volatile Integer Position = (0);
    private volatile Integer Instances = (0);
    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
    /**
     * Solver Constructor.
     * @throws InstantiationError When attempting to create more valid instances then what is allowed for this type
     */
    public Solver() throws InstantiationError {
      if(Instances > SOLVING_MAXIMUM_INSTANCES) {
        throw new InstantiationError();
      }
      Instances++;
    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Attempts for the result of a given reserved solvable instance, if it cannot be computed, empty is returned, limited
     * to 200 milliseconds for upon each run.
     */
    @Timeable(limit = 200, unit = TimeUnit.MILLISECONDS)
    public synchronized Optional<SolvableType> call() {
      try {
        synchronized(SOLVING_QUEUE) {
          while(!Thread.currentThread().isInterrupted() && !SOLVING_QUEUE.isEmpty()) { 
            Reserved = reserve();
            for(; Position < Reserved.HORIZON; Position++) {
              var Solve = solve((double) Position/(1000));
              if(Solve.isPresent()) {
                return Solve;
              }
            }
          }
          return Optional.empty();
        }
      } catch (final NoSuchElementException | InstanceAlreadyExistsException  Exception) {
        return Optional.empty();
      }
    }

    /**
     * Solves for a given type variable, at a given horizon
     * @param Horizon The current horizon point
     * @return Optional representing the solvable type
     */
    protected abstract Optional<SolvableType> solve(final Double Horizon);

    /**
     * Closes this instance and all held resources immediately.
     */
    @Override
    public synchronized void close() {
      SOLVING_QUEUE.clear();
    }

    /**
     * 
     */
    public synchronized CompletableFuture<SolvableType> submit(final Solvable<SolvableType> Object) {
      synchronized(SOLVING_QUEUE) {
        SOLVING_QUEUE.offer(Object);
      }
      return CompletableFuture.supplyAsync(
        () -> {

        }
      );
    }

    /**
     * Reserves a solvable instance from the queue
     * @return A reserved solvable instance from the queue
     * @throws InstanceAlreadyExistsException When attempting to 
     */
    @SuppressWarnings("unchecked")
    private synchronized Solvable<SolvableType> reserve() throws InstanceAlreadyExistsException {
      if(Reserved != null) {
        synchronized(SOLVING_QUEUE) {
          return (Solvable<SolvableType>) SOLVING_QUEUE.poll();
        }
      } else {
        throw new InstanceAlreadyExistsException();
      }
    }
  }

  /**
   * Represents a container for an object with a trajectory that has values which are unknown and need to be
   * solved for.
   */
  public static abstract class Solvable<SolvingType> {
    public Double MU;
    public Double MASS;    
    public Double HORIZON;
    public Double VERTICAL_AREA;
    public Double INITIAL_POSITION;    
    public Double INITIAL_VELOCITY;
    public Rotation2d INITIAL_ROTATION;    
    public Optional<SolvingType> SOLVING;
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