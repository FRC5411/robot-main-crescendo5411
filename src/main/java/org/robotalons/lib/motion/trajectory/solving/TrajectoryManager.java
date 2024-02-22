// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory.solving;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import java.io.Closeable;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;


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
  private static final List<TrajectorySolver> ACTIVE_SOLVERS;
  private static final Queue<Double> OUTPUT_QUEUE;

  private static final Integer OUTPUT_QUEUE_MAXIMUM_ELEMENTS = (300);
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static TrajectoryManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Trajectory Solver Constructor.
   */
  private TrajectoryManager() {
    start();
  } static {
    ACTIVE_SOLVERS = new ArrayList<>();
    OUTPUT_QUEUE = new ArrayBlockingQueue<>(OUTPUT_QUEUE_MAXIMUM_ELEMENTS);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public synchronized void run() {
    ACTIVE_SOLVERS.parallelStream().forEach((Solver) -> 
      Solver.call().ifPresent(OUTPUT_QUEUE::offer));
    
  }

  public synchronized void close() {

  }

  public synchronized void start() {
    super.start();
  }

  // public synchronized CompletableFuture<Double> submit(final TrajectoryObject Object) {
    
  // }
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
