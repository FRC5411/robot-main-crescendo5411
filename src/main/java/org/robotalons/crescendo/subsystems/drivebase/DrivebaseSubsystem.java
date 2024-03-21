// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Objects;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem.CameraIdentifier;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.sensors.Gyroscope;
import org.robotalons.lib.motion.utilities.OdometryThread;
import org.robotalons.lib.utilities.MathUtilities;
import org.robotalons.lib.utilities.Operator;
import org.robotalons.lib.utilities.Vector;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;
// ----------------------------------------------------------[Drivebase Subsystem]----------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseSubsystem</h1>
 *
 * <p>Utility class which controls the modules to achieve individual goal set points with an acceptable target range of accuracy and time
 * efficiency and providing an API for querying new goal states.<p>
 *
 * @see TalonSubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class DrivebaseSubsystem extends TalonSubsystemBase<Keybindings,Preferences,N1> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final List<SwerveModulePosition> MODULE_POSITIONS;
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SysIdRoutine CHARACTERIZATION_ROUTINE;
  private static final SwerveDriveKinematics KINEMATICS;
  private static final Notifier ODOMETRY_PROCESSOR;
  private static final Supplier<Boolean> ORIENTATION_FLIPPED;
  private static final List<Module> MODULES;
  private static final Gyroscope GYROSCOPE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Operator<Keybindings,Preferences> Operator;
  private static Rotation2d GyroscopeRotation;
  private static DrivebaseSubsystem Instance;
  private static DrivebaseState State;
  private static Double Timestamp;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */

  private DrivebaseSubsystem() {
    super(("Drivebase-Subsystem"), Nat.N1());
  } static {
    GYROSCOPE = Constants.Devices.GYROSCOPE;
    ORIENTATION_FLIPPED = () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red));
    State = DrivebaseState.ROBOT_ORIENTED;
    GyroscopeRotation = GYROSCOPE.getYawRotation();
    Timestamp = Logger.getRealTimestamp() / (1e6);
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
    final var Estimated = VisionSubsystem.getApproximatedRobotPose();
    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
      KINEMATICS, 
      GYROSCOPE.getYawRotation().plus(Rotation2d.fromDegrees((ORIENTATION_FLIPPED.get() && RobotBase.isSimulation())? (180d): (0d))), 
      getModulePositions(), 
      Estimated.isPresent()? Estimated.get().toPose2d(): new Pose2d()
    );
    MODULE_POSITIONS = new ArrayList<>();
    MODULES.stream().map(Module::getPosition).forEachOrdered(MODULE_POSITIONS::add);
    CHARACTERIZATION_ROUTINE = new SysIdRoutine(
      new SysIdRoutine.Config(
        (null),
        (null),
        (null),
        (State) -> Logger.recordOutput(("Drivebase-Subsystem/Module-Characterization"), State.toString())),
      new SysIdRoutine.Mechanism(
        (Voltage) -> MODULES.forEach(Module -> Module.characterize(Voltage.magnitude())),
        (null),
        getInstance())
    );
    ODOMETRY_PROCESSOR = new Notifier(DrivebaseSubsystem::process);
    ODOMETRY_PROCESSOR.setName(("Drivebase-Subsystem-Processor"));
    ODOMETRY_PROCESSOR.startPeriodic(((double) 1/(Measurements.ODOMETRY_FREQUENCY * OdometryThread.STANDARD_QUEUE_SIZE)));
    MODULES.forEach(Module::periodic);
    Logger.recordOutput(("Drivebase-Subsystem/Module-Measurements"), getModuleMeasurements());
    Logger.recordOutput(("Drivebase-Subsystem/Operator-State"), State.name());
    Logger.recordOutput(("Drivebase-Subsystem/Operator-Translation"), new Translation2d());
    Logger.recordOutput(("Drivebase-Subsystem/Operator-Rotation"), new Rotation2d());
    Logger.recordOutput(("Drivebase-Subsystem/Module-Reference"), getModuleUnoptimizedReferences());
    Logger.recordOutput(("Drivebase-Subsystem/Module-Optimized"), getModuleOptimizedReferences());
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Processes Module, Vision, and Gyroscope data from a single control loop of data collection & processing to create estimate
   * robot position. 
   */
  public static synchronized void process() {
    Objects.ODOMETRY_LOCK.lock();
    synchronized(MODULES) {
      final var Rotation = GYROSCOPE.getOdometryYawRotations();
      final var Measured = MODULES.stream().map(Module::getPositionDeltas).toList();
      final var Timestamps = MODULES.stream().map(Module::getPositionTimestamps).toList();
      final var Partitions = Timestamps.stream().mapToInt(List::size).boxed().toList();
      Partitions.stream().mapToInt(Integer::intValue).min().ifPresentOrElse((final int Size) -> {
        for(int Timestamp = (0); Timestamp < Math.min(Size, Rotation.length); Timestamp++) {
          try {
            final var Positions = new SwerveModulePosition[MODULES.size()];
            final var Deltas = new SwerveModulePosition[MODULES.size()];
            for (int Module = (0); Module < MODULES.size(); Module++) {
              Positions[Module] = Measured.get(Module).get(Timestamp);
              Deltas[Module] = new SwerveModulePosition(
                Positions[Module].distanceMeters - MODULE_POSITIONS.get(Module).distanceMeters,
                Positions[Module].angle);
              MODULE_POSITIONS.set(Module, Positions[Module]);
            } if (GYROSCOPE.getConnection() && Rotation.length != (0)) {
              GyroscopeRotation = Rotation[Timestamp];
            } else {
              final var Twist = KINEMATICS.toTwist2d(Deltas);
              GyroscopeRotation = GyroscopeRotation.plus(new Rotation2d(Twist.dtheta));
            }
            synchronized(POSE_ESTIMATOR) {
              POSE_ESTIMATOR.updateWithTime(
                Timestamps.get(Partitions.indexOf(Size)).get(Timestamp),
                GyroscopeRotation, 
                Positions
              );                    
            }
            if(Timestamp == (Math.min(Size, Rotation.length) - 1)) {
              Logger.recordOutput(("Drivebase-Subsystem/Odometry-Latency"), Timestamps.get(Partitions.indexOf(Size)).get(Timestamp) - Logger.getRealTimestamp() / (1e6));
            }
          } catch (final IndexOutOfBoundsException | NullPointerException Ignored) {
            break;
          } 
        }
      },
      () -> {
        synchronized(POSE_ESTIMATOR) {
          POSE_ESTIMATOR.updateWithTime(
            Logger.getRealTimestamp() / (1e6),
            GyroscopeRotation, 
            getModulePositions()
          );          
        }
      });
    }
    synchronized(POSE_ESTIMATOR) {
      VisionSubsystem.ALL_CAMERA_IDENTIFIERS.parallelStream().forEach((Identifier) -> {
        final var Timestamps = VisionSubsystem.getRobotPositionTimestamps(Identifier);
        final var Positions = VisionSubsystem.getRobotPositionDeltas(Identifier);
        final var Deviation = VisionSubsystem.getStandardDeviations(Identifier);
        IntStream.range((0), Math.min(Timestamps.length, Positions.length)).parallel().forEach((Index) -> POSE_ESTIMATOR.addVisionMeasurement(
          Positions[Index],
          Timestamps[Index],
          Deviation
        ));
      });      
    }
    Objects.ODOMETRY_LOCK.unlock();
  }


  @Override
  public synchronized void periodic() {
    MODULES.forEach(Module::periodic);
    GYROSCOPE.update();
    Logger.recordOutput("Drivebase-Subsystem/Module-Connection",MODULES.stream().allMatch(Module::getConnection));
    Logger.recordOutput(("Drivebase-Subsystem/Module-Measurements"),getModuleMeasurements());
    Logger.recordOutput(("Drivebase-Subsystem/Odometry-Estimation"), getPose());
    if (DriverStation.isDisabled()) {
      MODULES.forEach(Module::cease);
    }
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  private static synchronized Double discretize() {
    var Discretized = (0d);
    if (Timestamp.equals((0d))) {
      Discretized = ((1d) / (50d));
    } else {
      var Measured = Logger.getRealTimestamp() / (1e6);
      Discretized = Measured - Timestamp;
      Timestamp = Measured;
    }
    return Discretized;
  }

  /**
   * Resets the modules are pose estimation
   */
  public static synchronized void reset() {
    synchronized(MODULES) {
      GYROSCOPE.reset();
      MODULES.forEach(Module::reset);      
    }
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  @Override
  public synchronized void close() {
    ODOMETRY_PROCESSOR.stop();
    ODOMETRY_PROCESSOR.close();
    Instance = (null);
  }

  /**
   * Toggles between the possible states of orientation types
   */
  public static synchronized void toggle() {
    State = switch(State) {
      case ROBOT_ORIENTED -> DrivebaseState.FIELD_ORIENTED;
      case FIELD_ORIENTED -> DrivebaseState.OBJECT_ORIENTED;
      case OBJECT_ORIENTED -> DrivebaseState.ROBOT_ORIENTED;
    };
    Logger.recordOutput(("Drivebase-Subsystem/State"), State.name());
  }

  /**
   * Provides implementation for creating repeatable pose alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this pose autonomously
   */
  private static Command align(final Pose2d Source) {
    return SubsystemManager.pathfind(
      Source.transformBy(
      new Transform2d(
        getAlliance()? -Measurements.ROBOT_RADIUS_METERS: Measurements.ROBOT_RADIUS_METERS,
        (0d),
        new Rotation2d())),
      (0d)).beforeStarting(DrivebaseSubsystem::set, getInstance());
  }

  /**
   * Provides implementation for creating repeatable translation alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this translation autonomously
   */
  private static Command align(final Transform2d Source) {
    return SubsystemManager.pathfind(
      getPose().plus(
        new Transform2d(Source.getTranslation(), Source.getRotation())),
      (0d)).beforeStarting(DrivebaseSubsystem::set, getInstance());
  }

  @Override
  public void configureOperator(final Vector<Operator<Keybindings, Preferences>, N1> Operators) {
    DrivebaseSubsystem.Operator = Operators.DATA[0];
    getInstance().setDefaultCommand(
      new InstantCommand(() ->
      set((Operator.<Boolean>getRequiredPreference(Preferences.SQUARED_INPUT))?
          new Translation2d(
            -MathUtilities.signedSquare(MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.TRANSLATION_X_INPUT), Operator.<Double>getRequiredPreference(Preferences.TRANSLATIONAL_X_DEADZONE))),
            -MathUtilities.signedSquare(MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.TRANSLATION_Y_INPUT), Operator.<Double>getRequiredPreference(Preferences.TRANSLATIONAL_Y_DEADZONE)))):
          new Translation2d(
            -MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.TRANSLATION_X_INPUT), Operator.<Double>getRequiredPreference(Preferences.TRANSLATIONAL_X_DEADZONE)),
            -MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.TRANSLATION_Y_INPUT), Operator.<Double>getRequiredPreference(Preferences.TRANSLATIONAL_Y_DEADZONE))),
        (Operator.<Boolean>getRequiredPreference(Preferences.SQUARED_INPUT))?
          Rotation2d.fromRotations(
            -MathUtilities.signedSquare(MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.ORIENTATION_T_INPUT), Operator.<Double>getRequiredPreference(Preferences.ORIENTATION_DEADZONE)))):
          Rotation2d.fromRotations(
            -MathUtil.applyDeadband(Operator.<Double>getRequiredPreference(Preferences.ORIENTATION_T_INPUT),Operator.<Double>getRequiredPreference(Preferences.ORIENTATION_DEADZONE)))),
      getInstance()
    ));
    Operator.getOptionalKeybinding(Keybindings.ORIENTATION_TOGGLE).ifPresent((Trigger) -> 
      Trigger.onTrue(new InstantCommand(
        DrivebaseSubsystem::toggle,
        getInstance()
      )));
    Operator.getOptionalKeybinding(Keybindings.RESET_GYROSCOPE).ifPresent((Trigger) -> 
      Trigger.onTrue(new InstantCommand(
        DrivebaseSubsystem::reset,
        getInstance()
      )));
    Operator.getOptionalKeybinding(Keybindings.ALIGNMENT_SPEAKER).ifPresent((Trigger) -> 
      Trigger.whileTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getAprilTagPose(getAlliance()? (3): (7))
            .ifPresent((Pose) -> align(Pose.toPose2d()).schedule()),
        VisionSubsystem.getInstance(),
        getInstance()
      )));
    Operator.getOptionalKeybinding(Keybindings.ALIGNMENT_AMP).ifPresent((Trigger) -> 
      Trigger.whileTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getAprilTagPose((9))
          .ifPresent((Pose) -> align(Pose.toPose2d()).schedule()),
        VisionSubsystem.getInstance(),
        getInstance()
      )));

    Operator.getOptionalKeybinding(Keybindings.ALIGNMENT_OBJECT).ifPresent((Trigger) -> 
      Trigger.whileTrue(new InstantCommand(
        () -> 
        VisionSubsystem.getOptimalTarget(CameraIdentifier.INTAKE_CAMERA)
        .ifPresent((Transformation) -> align(new Transform2d(Transformation.getTranslation(), Transformation.getRotation())).schedule()),
        VisionSubsystem.getInstance(),
        getInstance()
      )));

    Operator.getOptionalKeybinding(Keybindings.ALIGNMENT_NEAREST).ifPresent((Trigger) -> 
      Trigger.whileTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getOptimalTarget(List.of(CameraIdentifier.SPEAKER_RIGHT_CAMERA, CameraIdentifier.SPEAKER_LEFT_CAMERA, CameraIdentifier.SPEAKER_RIGHT_CAMERA))
            .ifPresent((Transformation) -> align(new Transform2d(Transformation.getTranslation(), Transformation.getRotation())).schedule()),
        VisionSubsystem.getInstance(),
        getInstance()
    )));
  }

  /**
   * Performs linear characterization with sysID dynamically.
   * @param Direction Direction of travel to characterize in
   */
  public static synchronized void characterizeDynamic(final SysIdRoutine.Direction Direction) {
    MODULES.forEach(Module::cease);
    try {
      Thread.sleep((1000L));
    } catch(final InterruptedException Ignored) {}
    synchronized(CHARACTERIZATION_ROUTINE) {
      CHARACTERIZATION_ROUTINE.dynamic(Direction);
    }
  }

  /**
   * Performs linear characterization with sysID quasi-statically.
   * @param Direction Direction of travel to characterize in
   */
  public static synchronized void characterizeQausistatic(final SysIdRoutine.Direction Direction) {
    MODULES.forEach(Module::cease);
    try {
      Thread.sleep((1000L));
    } catch(final InterruptedException Ignored) {}
    synchronized(CHARACTERIZATION_ROUTINE) {
      CHARACTERIZATION_ROUTINE.quasistatic(Direction);
    }
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes a robot's current mode of orientation
   */
  public enum DrivebaseState {
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
    final var Discrete = ChassisSpeeds.discretize(Demand, discretize());
    final var Reference = KINEMATICS.toSwerveModuleStates(Discrete);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      Reference,
      Demand,
      Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
      Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
      Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY);
    Logger.recordOutput(("Drivebase-Subsystem/Odometry-Translation"), new Translation2d(Discrete.vxMetersPerSecond, Discrete.vyMetersPerSecond));
    Logger.recordOutput(("Drivebase-Subsystem/Odometry-Rotation"), new Rotation2d(Discrete.omegaRadiansPerSecond));
    set(List.of(Reference));
  }

  /**
   * Drives the robot provided translation and rotational demands, intended for joystick driven operation
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   */
  public static synchronized void set(final Translation2d Translation, final Rotation2d Rotation) {
    Logger.recordOutput(("Drivebase-Subsystem/Operator-Translation"), Translation);
    Logger.recordOutput(("Drivebase-Subsystem/Operator-Rotation"),Rotation);
    set(switch(State) {
      case FIELD_ORIENTED -> 
        ChassisSpeeds.fromFieldRelativeSpeeds(
          Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
          Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
          Rotation.getRotations() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY,
          getRotation());
      default -> 
        ChassisSpeeds.fromRobotRelativeSpeeds(
          Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
          Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
          Rotation.getRotations() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY,
          getRotation());          
    });
  }

  /**
   * Drives the robot provided Module States
   * @param States Collection of module states
   */
  public static synchronized void set(Collection<SwerveModuleState> States) {
    synchronized(MODULES) {
      final var Iterator = States.iterator();
      Logger.recordOutput(("Drivebase-Subsystem/Module-Reference"), States.toArray(SwerveModuleState[]::new));
      Logger.recordOutput(("Drivebase-Subsystem/Module-Optimized"), MODULES.stream().map((Module) -> Module.set(Iterator.next())).toArray(SwerveModuleState[]::new));      
    } 
  }

  /**
   * Stops all drivebase movement, if locking is enabled then all modules are
   * reset into an 'X' orientation.
   */
  public static synchronized void set() {
    MODULES.forEach(Module::reset);
  }

  /**
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static void set(final Pose2d Pose) {
    if(Pose != (null))  {
      synchronized(POSE_ESTIMATOR) {
        POSE_ESTIMATOR.resetPosition(GYROSCOPE.getYawRotation(), getModulePositions(), Pose);
      }
    }
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  @SuppressWarnings("unchecked")
  public Vector<Operator<Keybindings, Preferences>, N1> getOperators() {
    return new Vector<>(() -> (1), Operator);
  }
  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  public static synchronized Pose2d getPose() {
    synchronized(POSE_ESTIMATOR) {
      return POSE_ESTIMATOR.getEstimatedPosition();
    } 
  }

  /**
   * Provides a boolean representation of if the pathfinding should flip paths or not.
   * @return Boolean of if Pathfinding is flipped or not
   */
  public static Boolean getAlliance() {
    return ORIENTATION_FLIPPED.get();
  }

  /**
   * Provides a list of modules that this subsystem has
   * @return List of drive modules
   */
  public static List<Module> getModules() {
    return MODULES;
  }

  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  public static synchronized Rotation2d getRotation() {
    return GYROSCOPE.getConnection()? GYROSCOPE.getYawRotation(): getPose().getRotation();
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
  public static SwerveModuleState[] getModuleUnoptimizedReferences() {
    return MODULES.stream().map(
      Module::getReference
      ).toArray(SwerveModuleState[]::new);
  }

  public static SwerveModuleState[] getModuleOptimizedReferences() {
    return MODULES.stream().map(
      Module::getOptimized
      ).toArray(SwerveModuleState[]::new);
  }


  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  public static SwerveModuleState[] getModuleMeasurements() {
    return MODULES.stream().map(
      Module::getObserved
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output positions of all modules on the drivebase
   * @return Array of module positions
   */
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