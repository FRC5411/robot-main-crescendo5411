// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[TargetStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in position of targets. 
 */
@AutoLog
public class TargetStatusContainer {
  //TODO: Return robot pose, un edit this and change the abstract method once drivebase pose estimation implemented
  // public Pose3d BestTargetPose = new Pose3d(0, 0, 0, null);
  public Transform3d OptimalTransform = new Transform3d(0, 0, 0, null);
  public Transform3d[] Targets = new Transform3d[]{};
  public boolean HasTargets = (false);
  public int TotalTargets = 0;
}