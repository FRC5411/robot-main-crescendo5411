// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

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
  public boolean ContainsTarget = (false);
  public double Latency = (0d);
  public Pose3d RobotPose = new Pose3d(0, 0, 0, null);
  public Pose3d TargetPose = new Pose3d(0, 0, 0, null);
  public Transform3d BestTargetTransform = new Transform3d(0, 0, 0, null);

}