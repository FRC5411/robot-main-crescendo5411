// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory.solving;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import com.jcabi.aspects.Timeable;

import java.io.Closeable;
import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;
// -----------------------------------------------------------[Trajectory Solver]----------------------------------------------------------//
/**
 * 
 *
 * <h1>TrajectorySolver</h1>
 * 
 * <p>Solves upon a given object which is in projectile motion (free fall with initial velocity).
 * 
 * @see Callable
 * @see Closeable
 */
public class TrajectorySolver implements Callable<Optional<TrajectoryObject>>, Closeable{
  // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Integer SOLVING_QUEUE_MAXIMUM_ELEMENTS = (20);
  private static final Boolean SOLVING_QUEUE_IS_ORDERED = (true);
  private static final Integer SOLVING_MAXIMUM_INSTANCES = (10);
  private static final Queue<TrajectoryObject> SOLVING_QUEUE = new ArrayBlockingQueue<>(SOLVING_QUEUE_MAXIMUM_ELEMENTS, SOLVING_QUEUE_IS_ORDERED);

  private final static Double STANDARD_ACCELERATION = (-9.80665d);
  private final static Double STANDARD_DELTA = (1e-4);
  // -------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static volatile Integer Instances = (0);
  private volatile TrajectoryObject Reserved;
  private volatile Rotation2d Rotation;  
  private volatile Double Position;
  private volatile Integer Iteration;
  private volatile Double VelocityConstant;
  private volatile Double HorizonConstant;
  // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Solver Constructor.
   * @throws InstantiationError When attempting to create more valid instances than what is allowed for this type
   */
  public TrajectorySolver() throws InstantiationError {
    if(Instances > SOLVING_MAXIMUM_INSTANCES) {
      throw new InstantiationError();
    }
    Instances++;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  /**
   * Attempts for the result of a given reserved solvable instance, if it cannot be computed, empty is returned, limited
   * to 200 milliseconds for upon each run.
   * @return Optimized initial firing angle in radians
   */
  @Timeable(limit = 200, unit = TimeUnit.MILLISECONDS)
  public synchronized Optional<TrajectoryObject> call() {
    try {
      synchronized(SOLVING_QUEUE) {
        while(!Thread.currentThread().isInterrupted()) {
          if(Reserved == (null)) {
            synchronized(SOLVING_QUEUE) {
              Reserved = SOLVING_QUEUE.poll();
              Position = Reserved.OFFSET_LENGTH * Reserved.INITIAL_ROTATION.getCos();
            }              
            Rotation = Reserved.INITIAL_ROTATION;
          }
          final var Distance = Reserved.HORIZON - Position;
          for(                                          ; 
                                            Iteration < Reserved.ITERATIONS 
                                                        && 
            Math.abs(rotationalTrajectory(Distance, Reserved.INITIAL_VELOCITY, Rotation)) > STANDARD_DELTA;
                              Iteration++, Position = Iteration * (Reserved.HORIZON / Reserved.ITERATIONS) + (Reserved.OFFSET_LENGTH * Reserved.INITIAL_ROTATION.getCos())
          ) {
            Rotation = new Rotation2d(
              rootTangent(
                Rotation.getRadians(),
                rotationalTrajectory(Distance, Reserved.INITIAL_VELOCITY, Rotation),
                rotationalDiscreteDerivative(Distance, Reserved.INITIAL_VELOCITY, Rotation, new Rotation2d(STANDARD_DELTA))
              )
            );
          }
          Reserved.OPTIMIZED_ROTATION = Rotation;
          return Optional.of(Reserved);
        }
        return Optional.empty();
      }
    } catch (final NoSuchElementException | NullPointerException Exception) {
      Reserved = (null);
      return Optional.empty();
    }
  }

  /**
   * Closes this instance and all held resources immediately, does not guarantee execution of remaining elements within queue
   * @throws IOException When trying to close an instance that still has elements within the queue
   */
  public void close() throws IOException {
    if(!SOLVING_QUEUE.isEmpty()) {
      throw new IOException();
    }
    Reserved = (null);
    SOLVING_QUEUE.clear();
  }
  // ------------------------------------------------------------[Math Operations]-------------------------------------------------------------// 
  /**
   * Calculates the rotational trajectory at a given point along the horizon
   * @param Distance Current point along the trajectory
   * @param Velocity Current velocity along the trajectory
   * @param Rotation Current rotation along the trajectory
   * @return Trajectory at this point along the horizontal axis
   */
  private synchronized Double rotationalTrajectory(final Double Distance, final Double Velocity, final Rotation2d Rotation) {
    set(Distance, Velocity, Rotation);
    return HorizonConstant * Rotation.getTan()
    - Math.pow(HorizonConstant, (2)) * VelocityConstant * secantSquared(Rotation)
    + Reserved.OFFSET_LENGTH * Rotation.getSin()
    + Reserved.VERTICAL;
  }

  /**
   * Calculates the derivative discretely at a given point along the horizon
   * @param Distance Current point along the trajectory
   * @param Velocity Current velocity along the trajectory
   * @param Rotation Current rotation along the trajectory
   * @param Delta    Change in distance, i.e. the instantaneous change in distance to take the derivative of
   * @return Derivative at this point along the horizon
   */
  private synchronized Double rotationalDiscreteDerivative(final Double Distance, final Double Velocity, final Rotation2d Rotation, final Rotation2d Delta) {
    set(Distance, Velocity, Rotation);
    return (rotationalTrajectory(Distance, Velocity, Rotation.plus(Delta)) - rotationalTrajectory(Distance, Velocity, Rotation)) / Delta.getRadians();
  }

  /**
   * Calculates the derivative continuously at a given point along the horizon
   * @param Distance Current point along the trajectory
   * @param Velocity Current velocity along the trajectory
   * @param Rotation Current rotation along the trajectory
   * @return Derivative at this point along the horizon
   */
  @SuppressWarnings("unused")
  private synchronized Double rotationalContinuousDerivative(final Double Distance, final Double Velocity, final Rotation2d Rotation) {
    set(Distance, Velocity, Rotation);
    return (Math.PI / (180))
      * (HorizonConstant * secantSquared(Rotation)
          - (2)
              * Math.pow(HorizonConstant, (2))
              * VelocityConstant
              * secantSquared(Rotation)
              * Rotation.getTan()
          - Reserved.OFFSET_LENGTH * Rotation.getCos());
  }

  /**
   * Provides the tangent line of a given point at the roots of a function
   * @param X     Value of point along the x-axis
   * @param Y     Value of point along the y-axis
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
    return Math.pow((1) / Rotation.getCos(),(2));
  }
  
  /**
   * Mutates the current horizon constant given the current states of this object at a position.
   * @param Distance Current point along the trajectory
   * @param Velocity Current velocity along the trajectory
   * @param Rotation Current rotation along the trajectory
   */
  private synchronized void set(final Double Distance, final Double Velocity, final Rotation2d Rotation) {
    HorizonConstant = -(Reserved.HORIZON - Distance + Reserved.OFFSET_LENGTH * Rotation.getCos());
    VelocityConstant = STANDARD_ACCELERATION / ((2) * Velocity * Velocity);
  } 

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

  /**
   * Provides the length of the queue, as an integer; inclusive of the last index
   * @return Number of elements within the queue
   */
  public Integer getQueueLength() {
    return SOLVING_QUEUE.size();
  }
  
}
