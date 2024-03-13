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
 * <p>This file was converted into List and Stream(able) objects (lists) for ease of interfacing with data, from Mechanical Advantage's original
 * implementation found <a href="https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/example_projects/advanced_swerve_drive/src/main/java/frc/robot/subsystems/drive/PhoenixOdometryThread.java">here</a>.
 * 
 * @see OdometryThread
 * @see Thread
 * 
 * @author Cody Washington
 * @author Mechanical Advantage
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
  private static Boolean Enabled;
  private static Double Frequency;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Phoenix Odometry Thread Constructor.
   * @param Lock Appropriate Reentrance Locker for Odometry
   */
  private CTREOdometryThread(final Lock Lock) {
    ODOMETRY_LOCK = Lock;
    setName(this.getClass().getSimpleName());
    setDaemon((true));
    start();
  } static {
    SIGNAL_PROVIDERS = new ArrayList<>();
    SIGNAL_QUEUES = new ArrayList<>();
    TIMESTAMP_QUEUES = new ArrayList<>();
    SIGNALS_LOCK = new ReentrantLock();
    Frequency = STANDARD_FREQUENCY;
    Instance = (null);
    Flexible = (false);
    Enabled = (true);
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
    Queue<Double> Queue = new ArrayDeque<>(STANDARD_QUEUE_SIZE);
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
    SIGNALS_LOCK.lock();
    ODOMETRY_LOCK.lock();
    SIGNAL_QUEUES.clear();
    SIGNAL_PROVIDERS.clear();
    ODOMETRY_LOCK.unlock();
    SIGNALS_LOCK.unlock();
    interrupt();
    Instance = (null);
  }

  public synchronized Queue<Double> timestamp() {
    Queue<Double> Queue = new ArrayDeque<>(STANDARD_QUEUE_SIZE);
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
      if(!SIGNAL_PROVIDERS.isEmpty()) {
        try {
          SIGNALS_LOCK.lock();
          try {
            if (Flexible) {
              BaseStatusSignal.waitForAll((2d) / Frequency, SIGNAL_PROVIDERS.toArray(StatusSignal[]::new));
            } else {
              Thread.sleep((long) ((1000d) / Frequency));
              SIGNAL_PROVIDERS.forEach(StatusSignal::refresh);
            }
          } catch (final InterruptedException Ignored) {
            new Alert(("Odometry Interrupted"), AlertType.ERROR);
          } finally {
            SIGNALS_LOCK.unlock();
          }
          if(Enabled) {
            ODOMETRY_LOCK.lock();
            try {
              final var Providers = SIGNAL_PROVIDERS.iterator();
              final var Timestamp = new AtomicReference<>(Logger.getRealTimestamp() / (1e6));
              final var Latency = new AtomicReference<>((0d));
              SIGNAL_PROVIDERS.forEach((Signal) -> Latency.set(Latency.get() + Signal.getTimestamp().getLatency()));
              Timestamp.set(Timestamp.get() - Latency.get() / SIGNAL_PROVIDERS.size());
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
            }          
          } if(isInterrupted()) {
            break;
          }           
        } catch (final Exception Ignored) {
          new Alert(("Odometry Exception"), AlertType.ERROR);
        }         
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
   * @param Flexible If the CAN bus of devices is flexible
   */
  public synchronized void setFlexibility(final Boolean Flexible) {
    CTREOdometryThread.Flexible = Flexible;
  }

  @Override
  public synchronized void setEnabled(final Boolean Enabled) {
    CTREOdometryThread.Enabled = Enabled;
  }

  @Override
  public synchronized void setFrequency(final Double Frequency) {
    CTREOdometryThread.Frequency = Math.min(Frequency, (1000d));
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