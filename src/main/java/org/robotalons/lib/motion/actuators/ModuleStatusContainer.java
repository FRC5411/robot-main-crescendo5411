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
  public double LinearPositionRadians = (0d);
  public double LinearVelocityRadiansSecond = (0d);
  public double LinearAppliedVoltage = (0d);
  public double[] LinearCurrentAmperage = new double[] {};

  public Rotation2d RotationalAbsolutePosition = new Rotation2d();
  public Rotation2d RotationalRelativePosition = new Rotation2d();
  public double RotationalVelocityRadiansSecond = (0d);
  public double RotationalAppliedVoltage = (0d);
  public double[] RotationalAppliedAmperage = new double[] {};

  public double[] OdometryLinearPositionsRadians = new double[] {};
  public Rotation2d[] OdometryAzimuthPositions = new Rotation2d[] {};
}
