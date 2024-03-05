// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.trajectory.solving;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Rotation2d;

import java.io.Closeable;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
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
  private static final Queue<TrajectoryObject> OUTPUT_QUEUE;

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
    ACTIVE_SOLVERS.add(new TrajectorySolver());
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void run() {
    final var MultipleEmpty = new AtomicBoolean();
    while(this.isAlive()) {
      ACTIVE_SOLVERS.parallelStream().forEach((final TrajectorySolver Solver) -> {
        synchronized(Solver) {
          Solver.call().ifPresent(OUTPUT_QUEUE::offer);
          if (Solver.getQueueLength() == (0)) {
            MultipleEmpty.set(!MultipleEmpty.get());
            if(!MultipleEmpty.get()) {
              try {
                Solver.close();
              } catch (final IOException Ignored) {}
            }
          }          
        }
      });  
      MultipleEmpty.set((false));
    }
  }

  /**
   * Closes this instance and all held resources immediately, does not ensure the execution of remaining elements within the queue.
   */
  @Override
  public synchronized void close() {
    try {
      super.join();
    } catch (final InterruptedException Ignored) {}
    ACTIVE_SOLVERS.parallelStream().forEach((final TrajectorySolver Solver) -> {
      try {
        Solver.close();
      } catch (final IOException Ignored) {}
    });
  }

  /**
   * Submits a task object into the task queue
   * @param Object Task to be submitted, an unoptimized object instance
   * @return Completable result future, which supplies the optimized rotation as a result on completion
   */
  public CompletableFuture<Rotation2d> submit(final TrajectoryObject Object) {
    if(
                                                                        ACTIVE_SOLVERS.isEmpty() 
                                                                                    ^ 
      ACTIVE_SOLVERS.parallelStream().mapToInt(TrajectorySolver::getQueueLength).sum() / ACTIVE_SOLVERS.size() > TrajectorySolver.SOLVING_QUEUE_MAXIMUM_ELEMENTS)  {
        ACTIVE_SOLVERS.add(new TrajectorySolver());
    }
    return CompletableFuture.supplyAsync(() -> {
        final var Result = OUTPUT_QUEUE.stream().dropWhile((Solved) -> !Solved.equals(Object)).distinct().findFirst().get();
        OUTPUT_QUEUE.remove(Result);
        return Result.OPTIMIZED_ROTATION;
      }
    );
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
