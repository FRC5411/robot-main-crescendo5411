// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[TargetStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in position of targets. 
 */
@AutoLog
public class TargetStatusContainer  {
  public Pose3d BestTargetPose = new Pose3d(0, 0, 0, null);
  public Transform3d OptimalTransform = new Transform3d(0, 0, 0, null);
  public Transform3d[] Targets = new Transform3d[]{};
  public boolean HasTargets = (false);
  public int TotalTargets = 0;
  public boolean TargetDebounce = (false);
}