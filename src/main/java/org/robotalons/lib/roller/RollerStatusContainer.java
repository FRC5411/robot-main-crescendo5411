package org.robotalons.lib.roller;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
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
public class RollerStatusContainer {
  public double PositionRadians = (0d);
  public double VelocityRadiansSecond = (0d);
  public double AppliedVoltage = (0d);

}