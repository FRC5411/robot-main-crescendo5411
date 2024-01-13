// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.utilities;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.management.InstanceAlreadyExistsException;
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
public final class CTREOdometryThread extends OdometryThread<StatusSignal<Double>> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final Lock SIGNALS_LOCK = new ReentrantLock(); 
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static List<StatusSignal<Double>> Signals = new ArrayList<>();  
  private static CTREOdometryThread Instance = (null);    
  private static Boolean Is_Can_Flexible = (false);  
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Phoenix Odometry Thread Constructor.
   * @param OdometryLocker Appropriate Reentrance Locker for Odometry
   */
  private CTREOdometryThread(Lock OdometryLocker) {
    super(OdometryLocker);
    setName(("CTREOdometryThread"));
    setDaemon((true));
    start();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized Queue<Double> register(final StatusSignal<Double> Signal) {
    Queue<Double> Queue = new ArrayBlockingQueue<>(100);
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

  public synchronized void close() {
    QUEUES.clear();
    Signals.clear();    
    OdometryFrequency = (250d);    
    Instance = (null);
  }


  @Override
  public void run() {
    while (this.isAlive()) {
      SIGNALS_LOCK.lock();
      try {
        if (Is_Can_Flexible) {
          BaseStatusSignal.waitForAll((2.0) / OdometryFrequency, Signals.toArray(StatusSignal[]::new));
        } else {
          Thread.sleep((long) ((1000.0)/ OdometryFrequency));
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
  @Override
  public synchronized void setFrequency(final Double Frequency) {
    OdometryFrequency = Frequency;
  }

  /**
   * Mutates the current status of the can bus to determine if it supports flexible data rates.
   * @param IsFlexible If the CAN bus of devices is flexible
   */
  public synchronized void setCANFlexibleDataRate(final Boolean IsFlexible) {
    Is_Can_Flexible = IsFlexible;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Creates a new instance of the existing utility class
   * @return Utility class's instance
   * @throws InstanceAlreadyExistsException When the {@linkplain #create(Lock)} method has already been called prior to most-recent call
   */
  public static synchronized CTREOdometryThread create(Lock OdometryLock) throws InstanceAlreadyExistsException {
    if (!java.util.Objects.isNull(Instance)) {
      throw new InstanceAlreadyExistsException();
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