// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.TalonSubsystemBase;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Objects;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.pathfinding.LocalADStarAK;
import org.robotalons.lib.motion.sensors.Gyroscope;
import org.robotalons.lib.utilities.PilotProfile;

import java.util.List;
import java.util.stream.IntStream;
// ----------------------------------------------------------[Drivebase Subsystem]----------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseSubsystem</h1>
 *
 * <p>Utility class which controls the modules to achieve individual goal set points within an acceptable target range of accuracy and time 
 * efficiency and providing an API for querying new goal states.<p>
 * 
 * @see TalonSubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class DrivebaseSubsystem extends TalonSubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SwerveDriveKinematics KINEMATICS;
  private static final List<Module> MODULES;
  private static final Gyroscope GYROSCOPE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static List<SwerveModulePosition> CurrentPositions;
  private static OrientationMode CurrentMode;
  private static Rotation2d CurrentRotation;
  private static PilotProfile CurrentPilot;
  private static Pose2d CurrentPose;  
  private static Double CurrentTime;
  private static DrivebaseSubsystem Instance;
  private static Boolean ModuleLocked;      
  private static Boolean PathFlipped;    
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super("Drivebase Subsystem");
  } static {
    Instance = new DrivebaseSubsystem();
    CurrentPose = new Pose2d();
    GYROSCOPE = new PigeonGyroscope(Constants.Measurements.PHOENIX_DRIVE);
    CurrentMode = OrientationMode.ROBOT_ORIENTED;
    ModuleLocked = (true);
    PathFlipped = (false); 
    CurrentRotation = new Rotation2d();
    CurrentTime = Timer.getFPGATimestamp();
    MODULES = List.of(
      Devices.FRONT_LEFT_MODULE,
      Devices.FRONT_RIGHT_MODULE,
      Devices.REAR_LEFT_MODULE,
      Devices.REAR_RIGHT_MODULE
    );    
    KINEMATICS = new SwerveDriveKinematics(
      new Translation2d( (Constants.Measurements.ROBOT_WIDTH_METERS)  / (2), 
                         (Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d( (Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                         (Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(Constants.Measurements.ROBOT_LENGTH_METERS) / (2)));
    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
      KINEMATICS, 
      GYROSCOPE.getYawRotation(),
      getModulePositions(),   
      CurrentPose
    );
    AutoBuilder.configureHolonomic(
      DrivebaseSubsystem::getPose,
      DrivebaseSubsystem::set, 
      () -> KINEMATICS.toChassisSpeeds(getModuleMeasurements()), 
      DrivebaseSubsystem::set, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          Measurements.ROBOT_TRANSLATION_KP,
          Measurements.ROBOT_TRANSLATION_KI,
          Measurements.ROBOT_TRANSLATION_KP), 
        new PIDConstants(
          Measurements.ROBOT_ROTATIONAL_KP,
          Measurements.ROBOT_ROTATIONAL_KI,
          Measurements.ROBOT_ROTATIONAL_KD), 
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
        Measurements.ROBOT_RADIUS_METERS, 
        new ReplanningConfig(
          (true),
          (true)
        )), 
      () -> PathFlipped,
      Instance);
      Pathfinding.setPathfinder(new LocalADStarAK());
      PathPlannerLogging.setLogActivePathCallback(
        (ActivePath) -> Logger.recordOutput(("Drivebase/Trajectory"), ActivePath.toArray(new Pose2d[0])));
      PathPlannerLogging.setLogTargetPoseCallback(
        (TargetPose) -> Logger.recordOutput(("Drivebase/Reference"), TargetPose));
      MODULES.forEach((Module) -> 
        Module.set(org.robotalons.lib.motion.actuators.Module.ReferenceType.STATE_CONTROL));
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCK.lock();
    MODULES.forEach(Module::periodic);
    GYROSCOPE.update();    
    Logger.recordOutput(("Drivebase/Measurements"),getModuleMeasurements());
    if (DriverStation.isDisabled()) {
      MODULES.forEach(Module::cease);
    }
    Objects.ODOMETRY_LOCK.unlock();
    final var Timestamps = MODULES.get((0)).getPositionTimestamps();
    IntStream.range((0), Timestamps.size()).forEachOrdered((DeltaIndex) -> {
      SwerveModulePosition[] WheelDeltas = MODULES.stream().map(
        (Module) -> {
          final var Angle = Module.getPositionDeltas().get(DeltaIndex).angle;
          return new SwerveModulePosition(
              Module.getPositionDeltas().get(DeltaIndex).distanceMeters
                                          - 
                  CurrentPositions.get(DeltaIndex).distanceMeters,
            Angle);
        }).toArray(SwerveModulePosition[]::new);
      CurrentPositions = List.of(WheelDeltas);  
      if(GYROSCOPE.getConnected()) {
        CurrentRotation = GYROSCOPE.getOdometryYawRotations()[DeltaIndex];
      } else {
        Twist2d TwistDelta = KINEMATICS.toTwist2d(WheelDeltas);
        CurrentRotation = CurrentRotation.plus(new Rotation2d(TwistDelta.dtheta));
      }
      POSE_ESTIMATOR.updateWithTime(Timestamps.get(DeltaIndex), CurrentRotation, WheelDeltas);
    });
    //TODO: AUTOMATION TEAM (VISION MEASUREMENTS)
    org.robotalons.crescendo.Constants.Subsystems.ROBOT_FIELD.setRobotPose(getPose());
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  @AutoLogOutput(key = "Drivebase/DiscretizationTimestamp")
  private static synchronized Double discretize() {
    var DiscretizationTimestep = (0.0);
    if (CurrentTime.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Timer.getFPGATimestamp();
      DiscretizationTimestep = MeasuredTime - CurrentTime;
      CurrentTime = MeasuredTime;
    }    
    return DiscretizationTimestep;
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  public synchronized void close() {
    MODULES.forEach(Module::close);
    POSE_ESTIMATOR.resetPosition(
      GYROSCOPE.getYawRotation(),
      getModulePositions(),
      new Pose2d()
    );
  }

  /**
   * Toggles between the possible states of orientation types
   */
  public static synchronized void toggleOrientationType() {
    switch (CurrentMode) {
      case ROBOT_ORIENTED:
        CurrentMode = OrientationMode.FIELD_ORIENTED;
      case FIELD_ORIENTED:
        CurrentMode = OrientationMode.OBJECT_ORIENTED;
      case OBJECT_ORIENTED:
        CurrentMode = OrientationMode.ROBOT_ORIENTED;
    }
  }

  /**
   * Configures a pilot to operate this given subsystem.
   */
  public void configure(final PilotProfile Pilot) {
    CurrentPilot = Pilot;
    DrivebaseSubsystem.getInstance().setDefaultCommand(
      new InstantCommand(() ->
      DrivebaseSubsystem.set(
        new Translation2d(
          applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_INPUT),
        (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE))),
          applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_INPUT),
        (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE)))),
        new Rotation2d(
          applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.ORIENTATION_INPUT),
        (Double) CurrentPilot.getPreference(Preferences.ORIENTATION_DEADZONE))))), 
        DrivebaseSubsystem.getInstance()
    ));
    CurrentPilot.getKeybinding(Keybindings.ORIENTATION_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::toggleOrientationType, DrivebaseSubsystem.getInstance()));
    CurrentPilot.getKeybinding(Keybindings.MODULE_LOCKING_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::toggleModuleLocking, DrivebaseSubsystem.getInstance()));
    CurrentPilot.getKeybinding(Keybindings.PATHFINDING_FLIP_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::togglePathFlipped, DrivebaseSubsystem.getInstance()));
  }

  /**
   * Applies squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  private static Double applySquared(final Double Input) {
    return Math.copySign(Input * Input, Input);
  }

  /**
   * Toggles between the modules should go into a locking format when idle or not
   */
  public static synchronized void toggleModuleLocking() {
    ModuleLocked = !ModuleLocked;
  }

  /**
   * Toggles between is pathfinding should be flipped or not.
   */
  public static synchronized void togglePathFlipped() {
    PathFlipped = !PathFlipped;
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  public enum OrientationMode {
    OBJECT_ORIENTED,    
    ROBOT_ORIENTED,
    FIELD_ORIENTED,
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Drives the robot provided a chassis speeds demand
   * @param Demand Chassis speeds object which represents the demand speeds of the drivebase
   */
  public static synchronized void set(final ChassisSpeeds Demand) {
    if (Demand.omegaRadiansPerSecond > (1e-6) && Demand.vxMetersPerSecond > (1e-6) && Demand.vyMetersPerSecond > (1e-6) && ModuleLocked) {
      set();
    } else {
      var Discrete = ChassisSpeeds.discretize(Demand, discretize());
      var Reference = KINEMATICS.toSwerveModuleStates(Discrete);
      SwerveDriveKinematics.desaturateWheelSpeeds(
        Reference, 
        Discrete,
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
        Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY);
      Logger.recordOutput(("Drivebase/ReferenceTranslation"), new Translation2d(Discrete.vxMetersPerSecond, Discrete.vyMetersPerSecond));
      Logger.recordOutput(("Drivebase/ReferenceRotation"), new Rotation2d(Discrete.omegaRadiansPerSecond));
      Logger.recordOutput(("Drivebase/Reference"), Reference);
      Logger.recordOutput(("Drivebase/Optimized"),
        IntStream.range((0), MODULES.size()).boxed().map(
          (Index) -> {
            Reference[Index].speedMetersPerSecond *= (Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY * 10);
            return MODULES.get(Index).set(Reference[Index]);
          })
          .toArray(SwerveModuleState[]::new));
    }
  }

  /**
   * Drives the robot provided translation and rotational demands
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   */
  public static synchronized void set(final Translation2d Translation, final Rotation2d Rotation) {
    switch(CurrentMode) {
      case OBJECT_ORIENTED:
        //TODO: AUTOMATION TEAM (OBJECT ORIENTATION DRIVEBASE)
        break;      
      case ROBOT_ORIENTED:
        set(new ChassisSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians()));      
        break;
      case FIELD_ORIENTED:
        set(ChassisSpeeds.fromFieldRelativeSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians(), 
          GYROSCOPE.getYawRotation()));      
        break;
    }
  }

  /**
   * Stops all drivebase movement, if locking is enabled then all modules are 
   * reset into an 'X' orientation.
   */
  public static synchronized void set() {
    if(ModuleLocked) {
      set(new Translation2d(), new Rotation2d(Units.degreesToRadians((45))));
    } else {
      set(new ChassisSpeeds());
    }
  }

  /**
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static synchronized void set(final Pose2d Pose) {
    CurrentPose = Pose;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the current pilot of the drivebase
   * @return Pilot of this subsystem
   */
  public PilotProfile getPilot() {
    return CurrentPilot;
  }
  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  @AutoLogOutput(key = "Drivebase/Pose")
  public static Pose2d getPose() {
    return POSE_ESTIMATOR.getEstimatedPosition();
  }

  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  @AutoLogOutput(key = "Drivebase/Rotation")
  public static Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Provides the current chassis speeds
   * @return Chassis speeds of Robot drivebase
   */
  public static ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleMeasurements());
  }

  /**
   * Provides the current un-optimized reference ,'set-point' state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/Reference")
  public static SwerveModuleState[] getModuleReferences() {
    return MODULES.stream().map(
      Module::getReference
      ).toArray(SwerveModuleState[]::new);
  }


  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/Measurements")
  public static SwerveModuleState[] getModuleMeasurements() {
    return MODULES.stream().map(
      Module::getObserved
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output positions of all modules on the drivebase
   * @return Array of module positions
   */
  @AutoLogOutput(key = "Drivebase/Positions")
  public static SwerveModulePosition[] getModulePositions() {
    return MODULES.stream().map(
      Module::getPosition
      ).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
   public static synchronized DrivebaseSubsystem getInstance() {
    if (java.util.Objects.isNull(Instance)) {
      Instance = new DrivebaseSubsystem();
    }
    return Instance;
  }
}