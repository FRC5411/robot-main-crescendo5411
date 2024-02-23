// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.Notifier;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;
import javax.management.InstanceNotFoundException;
// ----------------------------------------------------------[REV Odometry Thread]----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues. This version is intended for devices
 * like the SparkMax that require polling rather than a blocking thread. A Notifier thread is used to gather samples with consistent timing.
 * 
 * @see OdometryThread
 * 
 */
public final class REVOdometryThread implements OdometryThread<DoubleSupplier> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<DoubleSupplier> SIGNALS;
  private static final List<Queue<Double>> TIMESTAMPS;
  private static final List<Queue<Double>> QUEUES;
  private final Lock ODOMETRY_LOCK;
  private final Notifier NOTIFIER;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static REVOdometryThread Instance;
  private static Double Frequency;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Odometry Thread Constructor.
   * @param OdometryLocker Appropriate Reentrance Locker for Odometry
   */
  private REVOdometryThread(Lock OdometryLocker) {
    ODOMETRY_LOCK = OdometryLocker;
    NOTIFIER = new Notifier(this);
    NOTIFIER.setName(("REVOdometryThread"));
    start();
  } static {
    QUEUES = new ArrayList<>();
    TIMESTAMPS = new ArrayList<>();
    SIGNALS = new ArrayList<>();
    Instance = (null);
    Frequency = (500d);
  }

  @Override
  public synchronized Queue<Double> register(final DoubleSupplier Signal) {
    Queue<Double> Queue = new ArrayDeque<>((100));
    ODOMETRY_LOCK.lock();
    try {
      SIGNALS.add(Signal);
      QUEUES.add(Queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  @Override
  public synchronized Queue<Double> timestamp() {
    Queue<Double> Queue = new ArrayDeque<>((100));
    ODOMETRY_LOCK.lock();
    try {
      TIMESTAMPS.add(Queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  public synchronized void start() {
    if (TIMESTAMPS.isEmpty()) {
      NOTIFIER.startPeriodic((1d)/ Frequency);
    }
  }

  @Override
  public synchronized void close() throws IOException {
    QUEUES.clear();
    SIGNALS.clear();
    NOTIFIER.stop();  
    Instance = (null);
  }

  @Override
  public synchronized void run() {
    ODOMETRY_LOCK.lock();
    try {
      final var SignalIterator = SIGNALS.iterator();
      final var RealTimestamp = Logger.getRealTimestamp() / (1e6);
      QUEUES.forEach((Queue) -> Queue.offer(SignalIterator.next().getAsDouble()));
      TIMESTAMPS.forEach((Timestamp) -> Timestamp.offer(RealTimestamp));
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }
  /**
   * Creates a new instance of the existing utility class
   * @return Utility class's instance
   */
  public static synchronized REVOdometryThread create(Lock OdometryLock) {
    if (!java.util.Objects.isNull(Instance)) {
      return Instance;
    }
    Instance = new REVOdometryThread(OdometryLock);
    return Instance;
  }

  @Override
  public Double getFrequency() {
    return Frequency;
  }  
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  
  @Override
  public synchronized void set(final Double Frequency) {
    ODOMETRY_LOCK.lock();
    REVOdometryThread.Frequency = Frequency;
    NOTIFIER.stop();
    NOTIFIER.startPeriodic(Frequency);
    ODOMETRY_LOCK.unlock();
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public Lock getLock() {
    return ODOMETRY_LOCK;
  }
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   * @throws InstanceNotFoundException When the {@linkplain #create(Lock) method has not yet been called}
   */
  public static synchronized REVOdometryThread getInstance() throws InstanceNotFoundException {
    if (java.util.Objects.isNull(Instance)) {
      throw new InstanceNotFoundException();
    }
    return Instance;
  }
}