// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Operators;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.Constants.Measurements;
import org.robotalons.crescendo.subsystems.climb.ClimbSubsystem;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.pathfinding.LocalADStarAK;
import org.robotalons.lib.utilities.TypeVector;

import java.io.Closeable;
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
@SuppressWarnings({"unchecked"})
public final class SubsystemManager extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final List<TalonSubsystemBase<Keybindings,Preferences,?>> SUBSYSTEMS;
  public static final TalonSubsystemBase<Keybindings,Preferences,?> SUPERSTRUCTURE_INSTANCE;  
  public static final TalonSubsystemBase<Keybindings,Preferences,?> DRIVEBASE_INSTANCE;
  public static final TalonSubsystemBase<Keybindings,Preferences,?> CLIMB_INSTANCE;
  public static final TalonSubsystemBase<Keybindings,Preferences,?> VISION_INSTANCE;
  public static final Field2d FIELD;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static volatile Command AutonomousScheduledCommand;
  private static volatile Boolean AutonomousMessagePrinted;
  private static volatile Double AutonomousStartTimestamp;
  private static SubsystemManager Instance;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private SubsystemManager() {} static {
    SUBSYSTEMS = new ArrayList<>();
    DRIVEBASE_INSTANCE = DrivebaseSubsystem.getInstance();
    SUPERSTRUCTURE_INSTANCE = SuperstructureSubsystem.getInstance();
    CLIMB_INSTANCE = ClimbSubsystem.getInstance();
    VISION_INSTANCE = VisionSubsystem.getInstance();
    SUBSYSTEMS.add(DRIVEBASE_INSTANCE);
    SUBSYSTEMS.add(SUPERSTRUCTURE_INSTANCE);
    SUBSYSTEMS.add(CLIMB_INSTANCE);
    SUBSYSTEMS.add(VISION_INSTANCE);
    VISION_INSTANCE.configure(new TypeVector<>(() -> (VISION_INSTANCE.getElements().getNum())));
    CLIMB_INSTANCE.configure(new TypeVector<>(() -> (CLIMB_INSTANCE.getElements().getNum()), Operators.Secondary.PROFILE));
    DRIVEBASE_INSTANCE.configure(new TypeVector<>(() -> (DRIVEBASE_INSTANCE.getElements().getNum()), Operators.Primary.PROFILE));
    SUPERSTRUCTURE_INSTANCE.configure(new TypeVector<>(() -> (SUPERSTRUCTURE_INSTANCE.getElements().getNum()), Operators.Secondary.PROFILE, Operators.Primary.PROFILE));
    SUBSYSTEMS.forEach(TalonSubsystemBase::configure);
    FIELD = new Field2d();
    AutoBuilder.configureHolonomic(
      DrivebaseSubsystem::getPose,
      DrivebaseSubsystem::set, 
      DrivebaseSubsystem::getChassisSpeeds,
      (final ChassisSpeeds Demand) -> DrivebaseSubsystem.set(Demand.unaryMinus()), 
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          Constants.Measurements.ROBOT_TRANSLATION_KP,
          Constants.Measurements.ROBOT_TRANSLATION_KI,
          Constants.Measurements.ROBOT_TRANSLATION_KD),
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
      () -> !DrivebaseSubsystem.getAlliance(),
      DRIVEBASE_INSTANCE);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
      (Trajectory) -> Logger.recordOutput(("Pathfinding/Trajectory"), Trajectory.toArray(Pose2d[]::new)));
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
    synchronized(FIELD) {
      FIELD.setRobotPose(DrivebaseSubsystem.getPose());
      if (AutonomousScheduledCommand != (null)) {
        if (!AutonomousScheduledCommand.isScheduled() && !AutonomousMessagePrinted) {
          System.out.printf(
            ("*** Auto %s in %.2f secs ***%n"),
            (DriverStation.isAutonomousEnabled()? "finished": "cancelled"),
            Logger.getRealTimestamp() / (1e6) - AutonomousStartTimestamp);
          AutonomousMessagePrinted = (true);
        }
      }      
    }
    //Pathfinding.setDynamicObstacles(new ArrayList<>(), DrivebaseSubsystem.getPose().getTranslation());
  }

  /**
   * Pathfinding and autonomously achieves the robot chassis to a give pose position; with a given end velocity
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

  /**
   * Pathfinding and autonomously achieves the robot chassis to a give transformed position; with a given end velocity
   * @param Transform Ending transform of the robot, the position for the chassis to achieve
   * @param Terminal  Ending velocity of the robot, the velocity for the chassis to end with
   * @return Command capable of pathfinding the robot to a given position
   */
  public static Command pathfind(final Transform2d Transform, final Double Terminal) {
    return pathfind(DrivebaseSubsystem.getPose().transformBy(Transform), Terminal);
  }

  /**
   * Cancels the currently scheduled autonomous command immediately.
   */
  public static synchronized void cancel() {
    if(AutonomousScheduledCommand != (null) && AutonomousScheduledCommand.isScheduled()) {
      AutonomousScheduledCommand.cancel();
    }
  }

  /**
   * Closes this instance and all held resources (subsystems) immediately.
   */
  public synchronized void close() {
    SUBSYSTEMS.forEach(TalonSubsystemBase::close);
    FIELD.close();
    Instance = (null);
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the currently running autonomous command
   * @param Autonomous New Autonomous command
   */
  public static synchronized void set(final Command Autonomous) {
    if(AutonomousScheduledCommand != (null) && AutonomousScheduledCommand.isScheduled()) {
      AutonomousScheduledCommand.cancel();
    }
    if(Autonomous != null) {  
      AutonomousMessagePrinted = (false);
      AutonomousStartTimestamp = Logger.getRealTimestamp() / (1e6);      
      AutonomousScheduledCommand = Autonomous;
      Autonomous.schedule();      
    }
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the employee subsystems managed my this subsystem manager
   * @return List of dependent subsystems
   */
  public static List<TalonSubsystemBase<Keybindings,Preferences,?>> getSubsystems() {
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