// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.Notifier;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;

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
 * <p>This file was converted into List and Stream(able) objects (lists) for ease of interfacing with data, from Mechanical Advantage's original
 * implementation found <a href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/advanced_swerve_drive/src/main/java/frc/robot/subsystems/drive/SparkMaxOdometryThread.java">here</a>.
 * 
 * @see OdometryThread
 * @see Notifier
 * 
 * @author Cody Washington
 * @author Mechanical Advantage
 * 
 */
public final class REVOdometryThread implements OdometryThread<DoubleSupplier> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<DoubleSupplier> SIGNAL_PROVIDERS;
  private static final List<Queue<Double>> TIMESTAMP_QUEUES;
  private static final List<Queue<Double>> SIGNAL_QUEUES;
  private final Lock ODOMETRY_LOCK;
  private final Notifier NOTIFIER;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static REVOdometryThread Instance;
  private static Double Frequency;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Odometry Thread Constructor.
   * @param Lock Appropriate Reentrance Locker for Odometry
   */
  private REVOdometryThread(final Lock Lock) {
    ODOMETRY_LOCK = Lock;
    NOTIFIER = new Notifier(this);
    NOTIFIER.setName(this.getClass().getSimpleName());
    start();
  } static {
    SIGNAL_QUEUES = new ArrayList<>();
    TIMESTAMP_QUEUES = new ArrayList<>();
    SIGNAL_PROVIDERS = new ArrayList<>();
    Instance = (null);
    Frequency = STANDARD_FREQUENCY;
  }

  @Override
  public synchronized Queue<Double> register(final DoubleSupplier Signal) {
    Queue<Double> Queue = new ArrayDeque<>((100));
    ODOMETRY_LOCK.lock();
    try {
      SIGNAL_PROVIDERS.add(Signal);
      SIGNAL_QUEUES.add(Queue);
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
      TIMESTAMP_QUEUES.add(Queue);
    } finally {
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  public synchronized void start() {
    if (TIMESTAMP_QUEUES.isEmpty()) {
      NOTIFIER.startPeriodic((1d)/ Frequency);
    }
  }

  @Override
  public synchronized void close() throws IOException {
    ODOMETRY_LOCK.lock();
    SIGNAL_QUEUES.clear();
    SIGNAL_PROVIDERS.clear();
    NOTIFIER.stop();  
    Instance = (null);
    ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void run() {
    try {
      ODOMETRY_LOCK.lock();
      try {
        final var Providers = SIGNAL_PROVIDERS.iterator();
        final var Timestamp = Logger.getRealTimestamp() / (1e6);
        SIGNAL_QUEUES.stream().forEachOrdered((final Queue<Double> Queue) -> {
          synchronized(Queue) {
            Queue.offer(Providers.next().getAsDouble());
          }
        });
        TIMESTAMP_QUEUES.stream().forEachOrdered((final Queue<Double> Queue) ->  {
          synchronized(Queue) {
            Queue.offer(Timestamp);
          }
        });
      } finally {
        ODOMETRY_LOCK.unlock();
      }
    } catch (final Exception Ignored) {
      new Alert(("Odometry Exception"), AlertType.ERROR);
    }
  }
  
  /**
   * Creates a new instance of the existing utility class
   * @param Lock Valid reentrance locker for this type
   * @return Utility class's instance
   */
  public static synchronized REVOdometryThread create(Lock Lock) {
    if (!java.util.Objects.isNull(Instance)) {
      return Instance;
    }
    Instance = new REVOdometryThread(Lock);
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