// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.sensors;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

// -------------------------------------------------------[Gyroscope Status Container]-------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the control of motion throughout the competition field.</p>
 */
@AutoLog
public class GyroscopeStatusContainer {

  public boolean Connected = (false);
  public Rotation2d YawRotation = new Rotation2d();
  public Rotation2d[] PositionDeltas = new Rotation2d[] {};
  public double YawVelocityRadiansSecond = (0d);
  
}
