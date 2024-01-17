// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.Notifier;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
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
    NOTIFIER = new Notifier(this::run);
    NOTIFIER.setName(("REVOdometryThread"));
    NOTIFIER.startPeriodic((1.0) / Frequency);
  } static {
    QUEUES = new ArrayList<>();
    SIGNALS = new ArrayList<>();
    Instance = (null);
    Frequency = (250d);
  }

  @Override
  public synchronized Queue<Double> register(final DoubleSupplier Signal) {
    Queue<Double> Queue = new ArrayBlockingQueue<>((100));
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
      var SignalIterator = SIGNALS.iterator();
      QUEUES.forEach((Queue) -> {
        Queue.offer(SignalIterator.next().getAsDouble());
      });
    } finally {
      ODOMETRY_LOCK.unlock();
    }
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  public synchronized void set(final Double Frequency) {
    REVOdometryThread.Frequency = Frequency;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
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