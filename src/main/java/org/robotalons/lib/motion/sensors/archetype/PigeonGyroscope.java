// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.sensors.archetype;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.sensors.Gyroscope;
import org.robotalons.lib.motion.utilities.OdometryThread;

import java.util.Queue;
// ------------------------------------------------------------[Pigeon Gyroscope]-----------------------------------------------------------//
/**
 *
 *
 * <h1>PigeonGyroscope</h1>
 *
 * <p>Implementation of an auto-logged Gyroscope using a Pigeon as hardware.<p>
 * 
 * @see Gyroscope
 * 
 * @author Cody Washington
 */
public class PigeonGyroscope extends Gyroscope {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Pigeon2 GYROSCOPE;
  private final StatusSignal<Double> YAW_ROTATION;
  private final StatusSignal<Double> YAW_VELOCITY;
  private final Queue<Double> YAW_ROTATION_QUEUE;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Pigeon Gyroscope Constructor.
   * @param Port Port (ID) of the PigeonDevice
   * @param Provider Status Provider which accepts Double StatusSignals 
   */
  public PigeonGyroscope(final Integer Port, final OdometryThread<StatusSignal<Double>> Provider) {
    GYROSCOPE = new Pigeon2(Port, "drivetrain/shooter");
    YAW_ROTATION = GYROSCOPE.getYaw();
    YAW_VELOCITY = GYROSCOPE.getAngularVelocityZWorld();
    GYROSCOPE.getConfigurator().apply(new Pigeon2Configuration());
    GYROSCOPE.getConfigurator().setYaw((0d));
    YAW_ROTATION.setUpdateFrequency(Provider.getFrequency());
    YAW_VELOCITY.setUpdateFrequency((100d));
    YAW_ROTATION_QUEUE = Provider.register(GYROSCOPE.getYaw());
    GYROSCOPE.optimizeBusUtilization();
    BaseStatusSignal.setUpdateFrequencyForAll((50), GYROSCOPE.getYaw());
  }

  public synchronized void close() {
    YAW_ROTATION_QUEUE.clear();
  }

  @Override
  public synchronized void update() {
    synchronized(STATUS) {
      STATUS.Connected = YAW_VELOCITY.refresh().getStatus() == StatusCode.OK;
      STATUS.YawRotation = Rotation2d.fromDegrees(YAW_ROTATION.getValue());
      STATUS.YawVelocityRadiansSecond = Units.degreesToRadians(YAW_VELOCITY.getValue());
      synchronized(YAW_ROTATION_QUEUE) {
        STATUS.PositionDeltas =
            YAW_ROTATION_QUEUE.stream()
                .map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);
        YAW_ROTATION_QUEUE.clear();    
      }  
      Logger.processInputs(("RealInputs/Gyroscope"), STATUS);
    }
  }
}
