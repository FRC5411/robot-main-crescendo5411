// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.robotalons.lib.motion.sensors.CommonGyroscope;
import org.robotalons.lib.utilities.OdometryThread;

import java.util.Queue;
// -------------------------------------------------------------[CTRE Gyroscope]------------------------------------------------------------//
/**
 *
 *
 * <h1>CTREGyroscope</h1>
 *
 * <p><p>
 * 
 * @see CommonGyroscope
 * @see DrivebaseSubsystem
 */
public class CTREGyroscope extends CommonGyroscope {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Pigeon2 GYROSCOPE = new Pigeon2(Constants.Ports.GYROSCOPE_ID);
  private final StatusSignal<Double> YAW_ROTATION = GYROSCOPE.getYaw();
  private final StatusSignal<Double> YAW_VELOCITY = GYROSCOPE.getAngularVelocityZ();
  private final Queue<Double> YAW_ROTATION_QUEUE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  public CTREGyroscope(final Boolean PhoenixDrive) {
    GYROSCOPE.getConfigurator().apply(new Pigeon2Configuration());
    GYROSCOPE.getConfigurator().setYaw((0.0));
    YAW_ROTATION.setUpdateFrequency(OdometryThread.OdometryFrequency);
    YAW_VELOCITY.setUpdateFrequency((100.0));
    if (PhoenixDrive) {
      YAW_ROTATION_QUEUE =
      org.robotalons.crescendo.Constants.Odometry.CTRE_ODOMETRY_THREAD.register(GYROSCOPE.getYaw());
    } else {
      YAW_ROTATION_QUEUE =
          org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD
              .register(() -> GYROSCOPE.getYaw().getValue());
    }

  }

  public synchronized void close() {
    YAW_ROTATION_QUEUE.clear();
  }

  @Override
  public synchronized void update() {
    Status.Connected = YAW_VELOCITY.refresh().getError() == StatusCode.OK;
    Status.YawRotation = Rotation2d.fromDegrees(YAW_ROTATION.getValue());
    Status.YawVelocityRadiansSecond = Units.degreesToRadians(YAW_VELOCITY.getValue());
    Status.PositionDeltas =
        YAW_ROTATION_QUEUE.stream()
            .map((Value) -> Rotation2d.fromDegrees(Value))
            .toArray(Rotation2d[]::new);
    YAW_ROTATION_QUEUE.clear();
  }
}
