// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;
import org.robotalons.lib.utilities.PilotProfile;

import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>RobotConstants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see RobotContainer
 */
public final class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//

  public static final class Subsystems {
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
    public static final Set<Subsystem> SUBSYSTEMS = Set.of(
      
    );
  }

  public static final class Logging {
    public static final String LOGGING_DEPOSIT_FOLDER = ("src\\main\\java\\main\\deploy\\logs");
    public static final Boolean LOGGING_TURBO_MODE = (false);
    public static final Boolean LOGGING_ENABLED = (false);
    public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Odometry {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
    public static final CTREOdometryThread CTRE_ODOMETRY_THREAD = CTREOdometryThread.create(ODOMETRY_LOCK);
    public static final REVOdometryThread REV_ODOMETRY_THREAD = REVOdometryThread.create(ODOMETRY_LOCK);
  }

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (0);
  }

  public static final class Profiles { 

    public static final List<PilotProfile> PILOT_PROFILES = List.of(
    
    );
  }
  public static final double highAngle = 0;
  public static final double idleAngle = 0;
  public static final double lowAngle = 0;
}
