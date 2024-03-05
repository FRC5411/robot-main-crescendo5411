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
  public double TranslationalPositionRadians = (0d);
  public double TranslationalVelocityRadiansSecond = (0d);
  public double TranslationalAppliedVoltage = (0d);
  public double TranslationalCurrentAmperage = (0d);
  public double TranslationTemperatureCelsius = (0d);

  public Rotation2d RotationalAbsolutePosition = new Rotation2d();
  public Rotation2d RotationalRelativePosition = new Rotation2d();
  public double RotationalVelocityRadiansSecond = (0d);
  public double RotationalAppliedVoltage = (0d);
  public double RotationalAppliedAmperage = (0d);
  public double RotationalTemperatureCelsius = (0d);

  public double[] OdometryTimestamps = new double[] {};
  public double[] OdometryTranslationalPositionsRadians = new double[] {};
  public Rotation2d[] OdometryRotationalPositionsRadians = new Rotation2d[] {};
}
