// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

// --------------------------------------------------------[Module Status Container]--------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the control of motion throughout the competition field.
 * 
 * @see Module
 */
@AutoLog
public class ModuleStatusContainer {
  public volatile double TranslationalPositionRadians = (0d);
  public volatile double TranslationalVelocityRadiansSecond = (0d);
  public volatile double TranslationalAppliedVoltage = (0d);
  public volatile double TranslationalCurrentAmperage = (0d);
  public volatile double TranslationalTemperatureCelsius = (0d);
  public volatile boolean TranslationalConnected = (false);

  public volatile Rotation2d RotationalAbsolutePosition = new Rotation2d();
  public volatile Rotation2d RotationalRelativePosition = new Rotation2d();
  public volatile double RotationalVelocityRadiansSecond = (0d);
  public volatile double RotationalAppliedVoltage = (0d);
  public volatile double RotationalCurrentAmperage = (0d);
  public volatile double RotationalTemperatureCelsius = (0d);
  public volatile boolean RotationalConnected = (false);

  public volatile double[] OdometryTimestamps = new double[] {};
  public volatile double[] OdometryTranslationalPositionsRadians = new double[] {};
  public volatile Rotation2d[] OdometryRotationalPositionsRadians = new Rotation2d[] {};
}
