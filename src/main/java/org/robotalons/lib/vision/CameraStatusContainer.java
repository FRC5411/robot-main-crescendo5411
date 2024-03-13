// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[CameraStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the odometry and kinematics of a given robot.
 */
@AutoLog
public class CameraStatusContainer {
  public boolean Connected = (false);
  public double Latency = (-1);
  public Pose2d Robot = new Pose2d();
  public Pose2d OptimalTargetPose = new Pose2d();
  public Transform2d OptimalTargetTransform = new Transform2d();
  public Transform2d[] RealizedTargets = new Transform2d[]{};
  public Pose2d[] Deltas = new Pose2d[]{};
  public double[] Timestamps = new double[]{};
} 