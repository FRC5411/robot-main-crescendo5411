// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose3d;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[CameraStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the odometry and kinematics of a given robot.
 */
@AutoLog
public class CameraStatusContainer {
  public String Name = ("");
  public boolean Connected = (false);
  public double Latency = (0d);
  public Pose3d RobotPose = new Pose3d(0, 0, 0, null);
  public ArrayList<Double> timestamps = new ArrayList<Double>();
  public ArrayList<Pose3d> deltas = new ArrayList<Pose3d>();
}