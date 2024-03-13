// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.AutoLogOutput;
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
import org.robotalons.lib.utilities.MathUtilities;
import org.robotalons.lib.utilities.Operator;

import java.util.ArrayList;
import java.util.Collection;
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
public class DrivebaseSubsystem extends TalonSubsystemBase<Keybindings,Preferences> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SysIdRoutine CHARACTERIZATION_ROUTINE;
  private static final SwerveDriveKinematics KINEMATICS;
  private static final Notifier ODOMETRY_PROCESSOR;
  private static final List<Module> MODULES;
  private static final Gyroscope GYROSCOPE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Operator<Keybindings,Preferences> Operator;
  private static List<SwerveModulePosition> ModulePositions;
  private static Rotation2d GyroscopeRotation;
  private static DrivebaseSubsystem Instance;
  private static DrivebaseState CurrentMode;
  private static Double Timestamp;
  private static Boolean FlippedEnabled;
  private static Boolean PrecisionEnabled;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {
    super(("Drivebase Subsystem"));
  } static {
    Instance = new DrivebaseSubsystem();
    GYROSCOPE = Constants.Devices.GYROSCOPE;
    CurrentMode = DrivebaseState.ROBOT_ORIENTED;
    FlippedEnabled = (DriverStation.getAlliance().isPresent()? (DriverStation.getAlliance().get().equals(Alliance.Red)): (false));
    GyroscopeRotation = GYROSCOPE.getYawRotation();
    Timestamp = Logger.getRealTimestamp() / (1e6);
    PrecisionEnabled = (false);
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
    final var Tag = org.robotalons.crescendo.subsystems.vision.Constants.Measurements.FIELD_LAYOUT.getTagPose((7)).get();
    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
      KINEMATICS, 
      GYROSCOPE.getYawRotation(), 
      getModulePositions(), 
      Estimated.isPresent()? 
      Estimated.get().toPose2d(): 
      new Pose2d(
        new Translation2d(
          Tag.getX() +
          (Measurements.ROBOT_RADIUS_METERS + 
          org.robotalons.crescendo.subsystems.superstructure.Constants.Measurements.OFFSET_WALL_METERS),
          Tag.getY()),
          GYROSCOPE.getYawRotation()));
    ModulePositions = new ArrayList<>();
    MODULES.stream().map(Module::getPosition).forEachOrdered(ModulePositions::add);
    CHARACTERIZATION_ROUTINE = new SysIdRoutine(
      new SysIdRoutine.Config(
        (null),
        (null),
        (null),
        (State) -> Logger.recordOutput(("Drivebase/Characterization"), State.toString())),
      new SysIdRoutine.Mechanism(
        (Voltage) -> {
          MODULES.forEach(Module -> Module.characterize(Voltage.magnitude()));
        },
        (null),
        getInstance())
    );
    ODOMETRY_PROCESSOR = new Notifier(DrivebaseSubsystem::process);
    ODOMETRY_PROCESSOR.setName(("OdometryProcessor"));
    ODOMETRY_PROCESSOR.startPeriodic((1/50));
    Logger.recordOutput(("Drivebase/Measurements"),getModuleMeasurements());
    Logger.recordOutput(("Drivebase/Translation"), new Translation2d());
    Logger.recordOutput(("Drivebase/Rotation"), new Rotation2d());
    Logger.recordOutput(("Drivebase/Reference"), new SwerveModuleState[MODULES.size()]);
    Logger.recordOutput(("Drivebase/Optimized"),new SwerveModuleState[MODULES.size()]);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Processes Module and Gyroscope data from a single control loop of module data
   */
  public static synchronized void process() {
    synchronized(MODULES) {
      final var Rotation = GYROSCOPE.getOdometryYawRotations();
      final var Measured = MODULES.stream().map(Module::getPositionDeltas).toList();
      final var Timestamps = MODULES.stream().map(Module::getPositionTimestamps).toList();
      final var Partitions = Timestamps.stream().mapToInt(List::size).boxed().toList();
      Partitions.stream().mapToInt(Integer::intValue).min().ifPresentOrElse((final int Size) -> {
        for(Integer Timestamp = (0); Timestamp < Math.min(Size, Rotation.length); Timestamp++) {
          try {
            final var Positions = new SwerveModulePosition[MODULES.size()];
            final var Deltas = new SwerveModulePosition[MODULES.size()];
            for (Integer Module = (0); Module < MODULES.size(); Module++) {
              Positions[Module] = Measured.get(Module).get(Timestamp);
              Deltas[Module] = new SwerveModulePosition(
                Positions[Module].distanceMeters - ModulePositions.get(Module).distanceMeters, 
                Positions[Module].angle);
              ModulePositions.set(Module, Positions[Module]);
            } if (GYROSCOPE.getConnected() && Rotation.length > (0)) {
              GyroscopeRotation = Rotation[Timestamp];
            } else {
              final var Twist = KINEMATICS.toTwist2d(Deltas);
              GyroscopeRotation = GyroscopeRotation.plus(new Rotation2d(Twist.dtheta));
            }
            POSE_ESTIMATOR.updateWithTime(
              Timestamps.get(Partitions.indexOf(Size)).get(Timestamp),
              GyroscopeRotation, 
              Positions
            );            
          } catch (final IndexOutOfBoundsException | NullPointerException Ignored) {
            break;
          } 
        }
      },
      () -> {
        POSE_ESTIMATOR.updateWithTime(
          Logger.getRealTimestamp() / (1e6),
          GyroscopeRotation, 
          getModulePositions()
        );
      });
    }
    VisionSubsystem.ALL_CAMERA_IDENTIFIERS.parallelStream().forEach((Identifier) -> {
      final var Timestamps = VisionSubsystem.getRobotPositionTimestamps(Identifier);
      final var Positions = VisionSubsystem.getRobotPositionDeltas(Identifier);
      final var Deviation = VisionSubsystem.getStandardDeviations(Identifier);
      IntStream.range((0), Math.min(Timestamps.length, Positions.length)).parallel().forEach((Index) -> {
        POSE_ESTIMATOR.addVisionMeasurement(
          Positions[Index],
          Timestamps[Index],
          Deviation
        );
      });
    });
  }


  @Override
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCK.lock();
    MODULES.forEach(Module::periodic);
    GYROSCOPE.update();
    Logger.recordOutput(("Drivebase/Measurements"),getModuleMeasurements());
    Logger.recordOutput(("Drivebase/Estimations"), getPose());
    if (DriverStation.isDisabled()) {
      MODULES.forEach(Module::cease);
    }
    Objects.ODOMETRY_LOCK.unlock();
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
  public synchronized void reset() {
    MODULES.forEach(Module::reset);
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  @Override
  public synchronized void close() {
    MODULES.forEach(Module::close);
    POSE_ESTIMATOR.resetPosition(
      GYROSCOPE.getYawRotation(),
      getModulePositions(),
      new Pose2d()
    );
    Instance = (null);
    ODOMETRY_PROCESSOR.stop();
    ODOMETRY_PROCESSOR.close();
  }

  /**
   * Toggles between the possible states of orientation types
   */
  public static synchronized void toggleState() {
    switch (CurrentMode) {
      case ROBOT_ORIENTED:
        CurrentMode = DrivebaseState.FIELD_ORIENTED;
        break;
      case FIELD_ORIENTED:
        CurrentMode = DrivebaseState.OBJECT_ORIENTED;
        break;
      case OBJECT_ORIENTED:
        CurrentMode = DrivebaseState.ROBOT_ORIENTED;
        break;
    }
  }

  /**
   * Toggles between the possible types of precision output
   */
  public static synchronized void togglePrecision() {
    PrecisionEnabled = !PrecisionEnabled;
  }

  /**
   * Provides implementation for creating repeatable pose alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this pose autonomously
   */
  public static Command alignPose(final Pose2d Source) {
    return SubsystemManager.pathfind(
      Source.transformBy(
      new Transform2d(
        getPath()? -Measurements.ROBOT_RADIUS_METERS: Measurements.ROBOT_RADIUS_METERS,
        (0d),
        new Rotation2d())),
      (0d));
  }

  /**
   * Provides implementation for creating repeatable translation alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this translation autonomously
   */
  public static Command alignTransformation(final Transform2d Source) {
    return SubsystemManager.pathfind(
      getPose().plus(
        new Transform2d(Source.getTranslation(), Source.getRotation())),
      (0d));
  }

  @Override
  public void configure(final Operator<Keybindings, Preferences> Profile) {
    Operator = Profile;
    DrivebaseSubsystem.getInstance().setDefaultCommand(
      new InstantCommand(() ->
      DrivebaseSubsystem.set(((Boolean) Operator.getPreference(Preferences.SQUARED_INPUT))?
        new Translation2d(
            MathUtilities.signedSquare(MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.TRANSLATION_X_INPUT),
          (Double) Operator.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE))),
            MathUtilities.signedSquare(MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.TRANSLATION_Y_INPUT),
          (Double) Operator.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE)))):
        new Translation2d(
          MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.TRANSLATION_X_INPUT),
        (Double) Operator.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE)),
          MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.TRANSLATION_Y_INPUT),
        (Double) Operator.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE))),
        ((Boolean) Operator.getPreference(Preferences.SQUARED_INPUT))?
        new Rotation2d(
            MathUtilities.signedSquare(MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.ORIENTATION_T_INPUT),
          (Double) Operator.getPreference(Preferences.ORIENTATION_DEADZONE)))):
        new Rotation2d(
          (MathUtil.applyDeadband((Double) Operator.getPreference(Preferences.ORIENTATION_T_INPUT),
        (Double) Operator.getPreference(Preferences.ORIENTATION_DEADZONE))))),
        DrivebaseSubsystem.getInstance()
    ));

    with(() ->
      Operator.getKeybinding(Keybindings.ORIENTATION_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::toggleState,
          DrivebaseSubsystem.getInstance()
        )));

    with(() ->
      Operator.getKeybinding(Keybindings.PRECISION_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::togglePrecision,
          DrivebaseSubsystem.getInstance()
        )));
  

    with(() ->
      Operator.getKeybinding(Keybindings.ALIGNMENT_SPEAKER)
      .onTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getAprilTagPose(getPath()? (3): (7))
            .ifPresent((Pose) -> {
                alignPose(Pose.toPose2d())
              .onlyWhile(() -> Operator.getKeybinding(Keybindings.ORIENTATION_TOGGLE).getAsBoolean()).schedule();
            }),
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      Operator.getKeybinding(Keybindings.ALIGNMENT_AMP)
      .onTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getAprilTagPose((9))
            .ifPresent((Pose) -> {
                alignPose(Pose.toPose2d())
              .onlyWhile(() -> Operator.getKeybinding(Keybindings.ORIENTATION_TOGGLE).getAsBoolean()).schedule();
            }),
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      Operator.getKeybinding(Keybindings.ALIGNMENT_OBJECT)
      .onTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getOptimalTarget(CameraIdentifier.INTAKE_CAMERA)
            .ifPresent((Transformation) -> {
              alignTransformation(new Transform2d(Transformation.getTranslation(), Transformation.getRotation()))
              .onlyWhile(() -> Operator.getKeybinding(Keybindings.ALIGNMENT_OBJECT).getAsBoolean()).schedule();
            }),
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      Operator.getKeybinding(Keybindings.ALIGNMENT_NEAREST)
      .onTrue(new InstantCommand(
        () -> 
          VisionSubsystem.getOptimalTarget(List.of(CameraIdentifier.SPEAKER_RIGHT_CAMERA, CameraIdentifier.SPEAKER_LEFT_CAMERA, CameraIdentifier.SPEAKER_RIGHT_CAMERA))
            .ifPresent((Transformation) -> {
                alignTransformation(new Transform2d(Transformation.getTranslation(), Transformation.getRotation()))
              .onlyWhile(() -> Operator.getKeybinding(Keybindings.ALIGNMENT_NEAREST).getAsBoolean()).schedule();
            }),
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));
  }

  /**
   * Performs linear characterization with sysID dynamically.
   * @param Direction Direction of travel to characterize in
   */
  public static synchronized void characterizeDynamic(final SysIdRoutine.Direction Direction) {
    synchronized(CHARACTERIZATION_ROUTINE) {
      CHARACTERIZATION_ROUTINE.dynamic(Direction);
    }
  }

  /**
   * Performs linear characterization with sysID quasi-statically.
   * @param Direction Direction of travel to characterize in
   */
  public static synchronized void characterizeQausistatic(final SysIdRoutine.Direction Direction) {
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
    var Discrete = ChassisSpeeds.discretize(Demand, discretize());
    var Reference = KINEMATICS.toSwerveModuleStates(Discrete);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      Reference, PrecisionEnabled? Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY * (2e-2): Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY);
    Logger.recordOutput(("Drivebase/Translation"), new Translation2d(Discrete.vxMetersPerSecond, Discrete.vyMetersPerSecond));
    Logger.recordOutput(("Drivebase/Rotation"), new Rotation2d(Discrete.omegaRadiansPerSecond));
    set(List.of(Reference));
  }

  /**
   * Drives the robot provided translation and rotational demands
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   */
  public static synchronized void set(final Translation2d Translation, final Rotation2d Rotation) {
    switch(CurrentMode) {
      case OBJECT_ORIENTED:
        VisionSubsystem.getOptimalTarget(CameraIdentifier.INTAKE_CAMERA).ifPresentOrElse((Optimal) -> {
          set(new ChassisSpeeds(
            -Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
            -Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
            (-(Math.PI) + Optimal.getRotation().getRadians() * (GYROSCOPE.getYawRotation().getRadians() % 2 * Math.PI > Math.PI? (-1): (1))) * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY
          ));
        }, () -> {
          CurrentMode = DrivebaseState.ROBOT_ORIENTED;
          set(Translation, Rotation);
        });
        break;
      case ROBOT_ORIENTED:
        set(new ChassisSpeeds(
          -Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          -Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          -Rotation.getRadians() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY
        ));
        break;
      case FIELD_ORIENTED:
        set(ChassisSpeeds.fromFieldRelativeSpeeds(
          -Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          -Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          -Rotation.getRadians() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY,
          GYROSCOPE.getYawRotation()
        ));
        break;
    }
  }

  /**
   * Drives the robot provided Module States
   * @param <State> State Type to apply from the module
   * @param States Collection of module states
   */
  public static synchronized <State extends SwerveModuleState> void set(Collection<State> States) {
    synchronized(MODULES) {
      final var StateIterator = States.iterator();
      Logger.recordOutput(("Drivebase/Reference"), States.toArray(SwerveModuleState[]::new));
      Logger.recordOutput(("Drivebase/Optimized"),
        MODULES.stream().map((Module) ->
          Module.set(StateIterator.next())).toArray(SwerveModuleState[]::new));      
    }
  }

  /**
   * Stops all drivebase movement, if locking is enabled then all modules are
   * reset into an 'X' orientation.
   */
  public static synchronized void set() {
    MODULES.forEach((Module) -> Module.reset());
  }

  /**
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static synchronized void set(final Pose2d Pose) {
    synchronized(POSE_ESTIMATOR) {
      POSE_ESTIMATOR.resetPosition(GYROSCOPE.getYawRotation(),getModulePositions(),getPose());
    }
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public Operator<Keybindings, Preferences> getOperator() {
    return Operator;
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
   * Provides a boolean representation of if the pathfinding should flip paths or not.
   * @return Boolean of if Pathfinding is flipped or not
   */
  public static Boolean getPath() {
    return FlippedEnabled;
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