// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

// -------------------------------------------------------[Trajectory Solver Manager]-------------------------------------------------------//
/**
 *
 * <h1>TrajectorySolverManager</h1>
 * 
 * <p>Manages a set of Trajectory Solvers to solve for a list of objects
 * 
 * @see Thread
 */
public class TrajectorySolverManager extends Thread {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final Double EQUIVALENCE_THRESHOLD = (1e-6);
  private static final Integer MINIMUM_SOLVERS = (1);
  private static final List<TrajectorySolver> WORKER_SOLVERS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectorySolverManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectorySolverManager() {
    start();
  } static {
    WORKER_SOLVERS = new ArrayList<TrajectorySolver>();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Starts the Manager, should not explicitly be called explicitly.
   * @throws IllegalAccessError When this method is called explicitly
   */
  @Override
  public synchronized void start() throws IllegalAccessError {
    //TODO: Initialize Solvers
    super.start();
  }


  /**
   * Operates upon the worker TrajectoryThreads, manages allocations, efficiency, etcetera
   */
  @Override
  public synchronized void run() {

  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * <h2>SolvableObject</h2>
   * 
   * Represents an object with the necessary values for a trajectory to be created
   */
  public static final class SolvableObject {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    // private final Function<Rotation2d,Double> HORIZONTAL_SURFACE_AREA;
    // private final Function<Rotation2d,Double> VERTICAL_SURFACE_AREA;
    // private final Integer HORIZON;
    // -------------------------------------------------------------[Fields]----------------------------------------------------------------//
    
    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//

    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//

  }

  /**
   *
   * <h1>TrajectorySolverManager</h1>
   * 
   * <p>Solves for the specific values of an object in real space.<p>
   * 
   * @see Thread
   */
  private static final class TrajectorySolver extends Thread implements Closeable {
    // ------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private static final Integer SOLVING_QUEUE_MAXIMUM = (20);
    private static final Double SOLVING_QUEUE_ACCURACY = (0.5d);
    private final Queue<SolvableObject> SOLVING_QUEUE;
    // -------------------------------------------------------------[Fields]----------------------------------------------------------------//

    // ----------------------------------------------------------[Constructors]-------------------------------------------------------------//
    /**
     * Trajectory Solver COnstructor.
     */
    public TrajectorySolver() {
      SOLVING_QUEUE = new ArrayBlockingQueue<>(SOLVING_QUEUE_MAXIMUM ,(false));
      start();
    } static {

    }
    // -------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Starts the Manager, should not explicitly be called explicitly.
     * @throws IllegalAccessError When this method is called explicitly
     */
    @Override
    public synchronized void start() throws IllegalAccessError {

      super.start();
    }

    @Override
    public synchronized void run() {
      synchronized(SOLVING_QUEUE) {
        final var Object = SOLVING_QUEUE.peek();
        //final var Timestep = SOLVING_QUEUE_ACCURACY * Object.HORIZON;
        
      }
    }

    /**
     * Closes this instance and all held resources immediately.
     */
    @Override
    public synchronized void close() {

    }
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
    /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized TrajectorySolverManager getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new TrajectorySolverManager();
      return Instance;
  }

}
