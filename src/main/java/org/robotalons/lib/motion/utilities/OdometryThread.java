// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.io.IOException;
import java.util.Queue;
// ------------------------------------------------------------[Odometry Thread]-----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues.<p>
 * 
 * @author Cody Washington
 * 
 */
public interface OdometryThread<SignalType extends Object> extends Runnable, Closeable {
  // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * Registers a new signal updated at a frequency with the frequency manager.
   * @param Signal Signal source, which can be queried for new signal values
   * @return The {@link Queue} of signal values
   */
  public abstract Queue<Double> register(final SignalType Signal);

  /**
   * Offers each relevant queue to the relevant signal value
   */
  public abstract void run();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   * @throws IOException When an Input Output operation has thrown an exception.
   */
  public abstract void close() throws IOException;

  /**
   * Mutates the current frequency of updating the odometry
   * @param Frequency Frequency of odometry updates in Hertz
   */
  public abstract void set(final Double Frequency);
}