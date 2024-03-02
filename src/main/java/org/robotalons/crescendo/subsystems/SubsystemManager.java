// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.climb.ClimbSubsystem;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem.CameraIdentifier;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.pathfinding.LocalADStarAK;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
// -----------------------------------------------------------[Subsystem Manager]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Subsystem Manager</h1>
 *
 * <p>Contains implementation for subsystems interfacing between each other to receive serialized information./p>
 *
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public final class SubsystemManager extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final List<TalonSubsystemBase> SUBSYSTEMS;
  public static final TalonSubsystemBase DRIVEBASE;
  public static final TalonSubsystemBase CANNON;
  public static final TalonSubsystemBase CLIMB;
  public static final TalonSubsystemBase VISION;
  public static final Field2d FIELD;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static SubsystemManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private SubsystemManager() {} static {
    SUBSYSTEMS = new ArrayList<>();
    FIELD = new Field2d();
    DRIVEBASE = DrivebaseSubsystem.getInstance();
    CANNON = SuperstructureSubsystem.getInstance();
    CLIMB = ClimbSubsystem.getInstance();
    VISION = VisionSubsystem.getInstance();
    //TODO: <... Fetch Other Subsystem Instances>
    SUBSYSTEMS.add(DRIVEBASE);
    SUBSYSTEMS.add(CANNON);
    SUBSYSTEMS.add(CLIMB);
    SUBSYSTEMS.add(VISION);
    //TODO: <... Add Other Subsystem Instances>
    AutoBuilder.configureHolonomic(
      DrivebaseSubsystem::getPose,
      DrivebaseSubsystem::set, 
      () -> DrivebaseSubsystem.getChassisSpeeds(),
      DrivebaseSubsystem::set, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          Constants.Measurements.ROBOT_TRANSLATION_KP,
          Constants.Measurements.ROBOT_TRANSLATION_KI,
          Constants.Measurements.ROBOT_TRANSLATION_KP), 
        new PIDConstants(
          Constants.Measurements.ROBOT_ROTATIONAL_KP,
          Constants.Measurements.ROBOT_ROTATIONAL_KI,
          Constants.Measurements.ROBOT_ROTATIONAL_KD), 
        Constants.Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
        Constants.Measurements.ROBOT_RADIUS_METERS, 
        new ReplanningConfig(
          (true),
          (true) 
        )), 
      () -> DrivebaseSubsystem.getPath(),
      DRIVEBASE);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
      (Trajectory) -> Logger.recordOutput(("Pathfinding/Trajectory"), Trajectory.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
      (Reference) -> Logger.recordOutput(("Pathfinding/Reference"), Reference));
    DrivebaseSubsystem.getModules().forEach((Module) -> 
      Module.set(org.robotalons.lib.motion.actuators.Module.ReferenceType.STATE_CONTROL));
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    Pathfinding.ensureInitialized();  
    FIELD.setRobotPose(DrivebaseSubsystem.getPose());
    Pathfinding.setDynamicObstacles(
      new ArrayList<>(),
      DrivebaseSubsystem.getPose().getTranslation());
  }

  /**
   * Ensures that the pathfinder is initialized, and that the start position of path finding is correct.
   */
  public static void ensure() {
    Pathfinding.ensureInitialized();
    Pathfinding.setStartPosition(DrivebaseSubsystem.getPose().getTranslation());
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the transform in three-dimensional space to the optimal target of a given camera
   * @param Port Which port to pull data from, ranging from 0, up to 4
   * @return Transformation in three-dimensional space camera relative
   */
  public static Optional<Transform3d> getOptimalTarget(final CameraIdentifier Port) {
    return VisionSubsystem.getOptimalTarget(Port);
  }

  /**
   * Provides the current chassis speeds
   * @return Chassis speeds of Robot drivebase
   */
  public static ChassisSpeeds getChassisSpeeds() {
    return DrivebaseSubsystem.getChassisSpeeds();
  }

  /**
   * Provides the employee subsystems managed my this subsystem manager
   * @return List of dependent subsystems
   */
  public static List<TalonSubsystemBase> getSubsystems() {
    return SUBSYSTEMS;
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized SubsystemManager getInstance() {
      if (java.util.Objects.isNull(Instance)) {
          Instance = new SubsystemManager();
      }
      return Instance;
  }
}
