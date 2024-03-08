// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.utilities.Alert;
import org.robotalons.lib.utilities.Alert.AlertType;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.management.InstanceNotFoundException;

// ----------------------------------------------------------[CTRE Odometry Thread]----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues. This version is intended for Phoenix
 * 6 devices on both the RIO and CANivore buses. When using a CANivore, the thread uses the "waitForAll" blocking method to enable more
 * consistent sampling. This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore time synchronization.
 * 
 * @see OdometryThread
 * @see Thread
 * 
 */
public final class CTREOdometryThread extends Thread implements OdometryThread<StatusSignal<Double>> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<StatusSignal<Double>> SIGNAL_PROVIDERS;
  private static final List<Queue<Double>> TIMESTAMP_QUEUES;  
  private static final List<Queue<Double>> SIGNAL_QUEUES;
  private static final Lock SIGNALS_LOCK;
  private final Lock ODOMETRY_LOCK;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static CTREOdometryThread Instance;  
  private static Boolean Flexible;
  private static Double Frequency;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Phoenix Odometry Thread Constructor.
   * @param OdometryLocker Appropriate Reentrance Locker for Odometry
   */
  private CTREOdometryThread(final Lock OdometryLocker) {
    ODOMETRY_LOCK = OdometryLocker;
    setName(("CTREOdometryThread"));
    setDaemon((true));
    start();
  } static {
    SIGNAL_PROVIDERS = new ArrayList<>();
    SIGNAL_QUEUES = new ArrayList<>();
    TIMESTAMP_QUEUES = new ArrayList<>();
    SIGNALS_LOCK = new ReentrantLock();
    Instance = (null);
    Frequency = (500d);
    Flexible = (false);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void start() {
    if (TIMESTAMP_QUEUES.isEmpty()) {
      super.start();
    }
  }
  
  @Override
  public synchronized Queue<Double> register(final StatusSignal<Double> Signal) {
    Queue<Double> Queue = new ArrayDeque<>((100));
    SIGNALS_LOCK.lock();
    ODOMETRY_LOCK.lock();
    try {
      SIGNAL_PROVIDERS.add(Signal);
      SIGNAL_QUEUES.add(Queue);
    } finally {
      SIGNALS_LOCK.unlock();
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  public synchronized void close() throws IOException {
    SIGNAL_QUEUES.clear();
    SIGNAL_PROVIDERS.clear();
    try {
      super.join();
    } catch (final InterruptedException Exception) {
      Exception.printStackTrace();
    }     
    Instance = (null);
    this.interrupt();
  }

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

  @Override
  public void run() {
    while (this.isAlive()) {
      try {
        SIGNALS_LOCK.lock();
        try {
          if (Flexible) {
            BaseStatusSignal.waitForAll((2d) / Frequency, SIGNAL_PROVIDERS.toArray(StatusSignal[]::new));
          } else {
            Thread.sleep((long) ((1000d) / Frequency));
            SIGNAL_PROVIDERS.forEach(StatusSignal::refresh);
          }
        } catch (final InterruptedException Exception) {
          Exception.printStackTrace();
        } finally {
          SIGNALS_LOCK.unlock();
        }
        ODOMETRY_LOCK.lock();
        try {
          final var Providers = SIGNAL_PROVIDERS.iterator();
          final var Timestamp = new AtomicReference<>(Logger.getRealTimestamp() / (1e6));
          final var Latency = new AtomicReference<>((0d));
          SIGNAL_PROVIDERS.forEach((Signal) -> Latency.set(Latency.get() + Signal.getTimestamp().getLatency()));
          if (!SIGNAL_PROVIDERS.isEmpty()) {
            Timestamp.set(Timestamp.get() - Latency.get() / SIGNAL_PROVIDERS.size());
          }
          SIGNAL_QUEUES.stream().forEachOrdered((final Queue<Double> Queue) -> {
            synchronized(Queue) {
              Queue.offer(Providers.next().getValue());
            }
          });
          TIMESTAMP_QUEUES.stream().forEachOrdered((final Queue<Double> Queue) ->  {
            synchronized(Queue) {
              Queue.offer(Timestamp.get());
            }
          });
        } finally {
          ODOMETRY_LOCK.unlock();
        } if(this.isInterrupted()) {
          break;
        }
      } catch (final Exception Ignored) {
        new Alert(("Odometry Exception"), AlertType.ERROR);
      }
    }
  }  

  /**
   * Creates a new instance of the existing utility class
   * @param Lock Valid reentrance locker for this type
   * @return Utility class's instance
   */
  public static synchronized CTREOdometryThread create(final Lock Lock) {
    if (!java.util.Objects.isNull(Instance)) {
      return Instance;
    }
    Instance = new CTREOdometryThread(Lock);
    return Instance;
  }


  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the current status of the can bus to determine if it supports flexible data rates.
   * @param IsFlexible If the CAN bus of devices is flexible
   */
  public synchronized void set(final Boolean IsFlexible) {
    Flexible = IsFlexible;
  }

  @Override
  public synchronized void set(final Double Frequency) {
    CTREOdometryThread.Frequency = Frequency;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public Lock getLock() {
    return ODOMETRY_LOCK;
  }

  @Override
  public Double getFrequency() {
    return Frequency;
  }  
  
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   * @throws InstanceNotFoundException When the {@linkplain #create(Lock) method has not yet been called}
   */
  public static synchronized CTREOdometryThread getInstance() throws InstanceNotFoundException {
    if (java.util.Objects.isNull(Instance)) {
      throw new InstanceNotFoundException();
    }
    return Instance;
  }
}