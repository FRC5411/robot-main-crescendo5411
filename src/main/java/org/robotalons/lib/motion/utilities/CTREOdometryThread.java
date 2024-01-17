// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.management.InstanceNotFoundException;

// ----------------------------------------------------------[CTRE Odometry Thread]----------------------------------------------------------//
/**
 *
 *
 * <p>Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 * 
 * @see OdometryThread
 * 
 */
public final class CTREOdometryThread extends Thread implements OdometryThread<StatusSignal<Double>> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<Queue<Double>> QUEUES;
  private static final Lock SIGNALS_LOCK;
  private final Lock ODOMETRY_LOCK;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static List<StatusSignal<Double>> Signals;
  private static CTREOdometryThread Instance;  
  private static Boolean FlexibleCAN;
  private static Double Frequency;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Phoenix Odometry Thread Constructor.
   * @param OdometryLocker Appropriate Reentrance Locker for Odometry
   */
  private CTREOdometryThread(Lock OdometryLocker) {
    ODOMETRY_LOCK = OdometryLocker;
    setName(("CTREOdometryThread"));
    setDaemon((true));
    start();
  } static {
    QUEUES = new ArrayList<>();
    SIGNALS_LOCK = new ReentrantLock();
    Instance = (null);
    Frequency = (250d);
    FlexibleCAN = (false);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized Queue<Double> register(final StatusSignal<Double> Signal) {
    Queue<Double> Queue = new ArrayBlockingQueue<>((100));
    SIGNALS_LOCK.lock();
    ODOMETRY_LOCK.lock();
    try {
      List<StatusSignal<Double>> UniqueSignals = new ArrayList<>();
      System.arraycopy(Signals, (0), UniqueSignals, (0), Signals.size());
      UniqueSignals.add(Signal);
      Signals = UniqueSignals;
      QUEUES.add(Queue);
    } finally {
      SIGNALS_LOCK.unlock();
      ODOMETRY_LOCK.unlock();
    }
    return Queue;
  }

  public synchronized void close() throws IOException {
    QUEUES.clear();
    Signals.clear();    
    FlexibleCAN = (false);
    try {
      super.join();
    } catch (final InterruptedException Exception) {
      Exception.printStackTrace();
    }     
    Instance = (null);
  }


  @Override
  public void run() {
    while (this.isAlive()) {
      SIGNALS_LOCK.lock();
      try {
        if (FlexibleCAN) {
          BaseStatusSignal.waitForAll((2.0) / Frequency, Signals.toArray(StatusSignal[]::new));
        } else {
          Thread.sleep((long) ((1000.0)/ Frequency));
          Signals.forEach(StatusSignal::refresh);
        }
      } catch (InterruptedException Exception) {
        Exception.printStackTrace();
      } finally {
        SIGNALS_LOCK.unlock();
      }
      ODOMETRY_LOCK.lock();
      try {
        var SignalIterator = Signals.iterator();
        QUEUES.forEach((Queue) -> {
          Queue.offer(SignalIterator.next().getValue());
        });
      } finally {
        ODOMETRY_LOCK.unlock();
      }
    }
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the current status of the can bus to determine if it supports flexible data rates.
   * @param IsFlexible If the CAN bus of devices is flexible
   */
  public synchronized void set(final Boolean IsFlexible) {
    FlexibleCAN = IsFlexible;
  }

  public synchronized void set(final Double Frequency) {
    CTREOdometryThread.Frequency = Frequency;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Creates a new instance of the existing utility class
   * @return Utility class's instance
   */
  public static synchronized CTREOdometryThread create(Lock OdometryLock) {
    if (!java.util.Objects.isNull(Instance)) {
      return Instance;
    }
    Instance = new CTREOdometryThread(OdometryLock);
    return Instance;
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