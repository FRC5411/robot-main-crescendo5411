// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.pathfinding.LocalADStarAK;

import java.util.ArrayList;
import java.util.List;
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
  public static final List<TalonSubsystemBase<Keybindings,Preferences>> SUBSYSTEMS;
  public static final TalonSubsystemBase<Keybindings,Preferences> DRIVEBASE;
  public static final TalonSubsystemBase<Keybindings,Preferences> CANNON;
  public static final TalonSubsystemBase<Keybindings,Preferences> VISION;
  public static final Field2d FIELD;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static SubsystemManager Instance;
  private static volatile Boolean AutonomousStatus = (false);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private SubsystemManager() {} static {
    SUBSYSTEMS = new ArrayList<>();
    FIELD = new Field2d();
    DRIVEBASE = DrivebaseSubsystem.getInstance();
    CANNON = SuperstructureSubsystem.getInstance();
    VISION = VisionSubsystem.getInstance();
    SUBSYSTEMS.add(DRIVEBASE);
    SUBSYSTEMS.add(CANNON);
    SUBSYSTEMS.add(VISION);
    AutoBuilder.configureHolonomic(
      DrivebaseSubsystem::getPose,
      DrivebaseSubsystem::set, 
      () -> DrivebaseSubsystem.getChassisSpeeds(),
      (final ChassisSpeeds Demand) -> DrivebaseSubsystem.set(Demand.times((-1))), 
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
    Pathfinding.ensureInitialized();
    Pathfinding.setStartPosition(DrivebaseSubsystem.getPose().getTranslation());
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public synchronized void periodic() {
    FIELD.setRobotPose(DrivebaseSubsystem.getPose());
    Pathfinding.setDynamicObstacles(
      new ArrayList<>(),
      DrivebaseSubsystem.getPose().getTranslation());
  
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Pathfinds and autonomously achieves the robot chassis to a give pose position; with a given end velocity
   * @param Pose     Ending pose of the robot, the position for the chassis to achieve
   * @param Terminal Ending velocity of the robot, the velocity for the chassis to end with
   * @return Command capable of pathfinding the robot to a given position
   */
  public static Command pathfind(final Pose2d Pose, final Double Terminal) {
    return AutoBuilder.pathfindToPoseFlipped(
      Pose,
      new PathConstraints(
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
        Measurements.ROBOT_MAXIMUM_LINEAR_ACCELERATION,
        Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY, 
        Measurements.ROBOT_MAXIMUM_ANGULAR_ACCELERATION
      ),
      Terminal
    );
  }

  public static synchronized void setAutonomousStatus(final Boolean Status) {
    AutonomousStatus = Status;
  }

  public static synchronized Boolean getAutonomousStatus() {
    return !AutonomousStatus;
  }

  public synchronized static void configureAutonomous(){
    NamedCommands.registerCommand("Intake Note", SuperstructureSubsystem.grabNote());
    NamedCommands.registerCommand("Shoot Subwoofer ", SuperstructureSubsystem.shoot(0.6));
    NamedCommands.registerCommand("Pivot Subwoofer", SuperstructureSubsystem.shootAtSubwoofer());
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
  public static List<TalonSubsystemBase<Keybindings, Preferences>> getSubsystems() {
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