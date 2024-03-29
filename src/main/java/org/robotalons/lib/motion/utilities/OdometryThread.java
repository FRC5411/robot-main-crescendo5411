// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.io.Closeable;
import java.io.IOException;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
// ------------------------------------------------------------[Odometry Thread]-----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues.<p>
 * 
 * @author Cody Washington
 * 
 * @see Runnable
 * @see Closeable
 * 
 * @author Cody Washington
 * 
 */
public sealed interface OdometryThread<SignalType> extends Runnable, Closeable permits CTREOdometryThread, REVOdometryThread {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final Double STANDARD_FREQUENCY = (250d);
  public static final Integer STANDARD_QUEUE_SIZE = (10);
  // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
  /**
   * Registers a new signal updated at a frequency with the frequency manager.
   * @param Signal Signal source, which can be queried for new signal values
   * @return The {@link Queue} of signal values
   */
  Queue<Double> register(final SignalType Signal);

  /**
   * Provides the timestamps for the available odometry queues
   * @return The {@link Queue} of timestamp values
   */
  Queue<Double> timestamp(); 

  /**
   * Offers each relevant queue to the relevant signal value
   */
  void run();

  /**
   * Closes this instance and all held resources immediately, but does not render the class unusable hence forth and can be re-instantiated.
   * @throws IOException When an Input Output operation has thrown an exception.
   */
  void close() throws IOException;

  /**
   * Mutates the current frequency of updating the odometry
   * @param Frequency Frequency of odometry updates in Hertz
   */
  void setFrequency(final Double Frequency);


  /**
   * Mutates the current state of the thread to be enabled or disabled, note that this has different behavior from {@link #close}, 
   * which stops this instance without the ability to re-enable it.
   * @param Enabled If this Thread is enabled or not
   */
  void setEnabled(final Boolean Enabled);

  /**
   * Provides the odometry lock of this thread
   * @return Lock which prevents method reentrance
   */
  Lock getLock();

  /**
   * Provides the frequency of this thread
   * @return Frequency as a double
   */
  Double getFrequency();
}