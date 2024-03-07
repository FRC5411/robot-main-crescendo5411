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
  private static final List<Module> MODULES;
  private static final Gyroscope GYROSCOPE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static List<SwerveModulePosition> CurrentPositions;
  private static DrivebaseState CurrentMode;
  private static Rotation2d CurrentRotation;
  private static Operator<Keybindings,Preferences> CurrentOperator;
  private static Double CurrentTime;
  private static DrivebaseSubsystem Instance;
  private static Boolean ModuleLocked;
  private static Boolean PathFlipped;
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
    ModuleLocked = (true);
    PathFlipped = (
      DriverStation.getAlliance().isPresent()?
       (DriverStation.getAlliance().get().equals(Alliance.Red)):
       (false)
    );
    CurrentRotation = GYROSCOPE.getYawRotation();
    CurrentTime = Logger.getRealTimestamp() / (1e6);
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
    if (Estimated.isPresent()) {
      POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
        KINEMATICS,
        GYROSCOPE.getYawRotation(),
        getModulePositions(),
        Estimated.get().toPose2d()
      );
    } else {
      POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
        KINEMATICS,
        GYROSCOPE.getYawRotation(),
        getModulePositions(),
        new Pose2d(new Translation2d(), CurrentRotation)
      );
    }
    CurrentPositions = new ArrayList<>();
    MODULES.stream().map(Module::getPosition).forEachOrdered(CurrentPositions::add);
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
    Logger.recordOutput(("Drivebase/Measurements"),getModuleMeasurements());
    Logger.recordOutput(("Drivebase/Translation"), new Translation2d());
    Logger.recordOutput(("Drivebase/Rotation"), new Rotation2d());
    Logger.recordOutput(("Drivebase/Reference"), new SwerveModuleState[MODULES.size()]);
    Logger.recordOutput(("Drivebase/Optimized"),new SwerveModuleState[MODULES.size()]);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
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
                Positions[Module].distanceMeters - CurrentPositions.get(Module).distanceMeters, 
                Positions[Module].angle);
              CurrentPositions.set(Module, Positions[Module]);;
            } if (GYROSCOPE.getConnected() && Rotation.length > (0)) {
              CurrentRotation = Rotation[Timestamp];
            } else {
              final var Twist = KINEMATICS.toTwist2d(Deltas);
              CurrentRotation = CurrentRotation.plus(new Rotation2d(Twist.dtheta));
            }
            POSE_ESTIMATOR.updateWithTime(
              Timestamps.get(Partitions.indexOf(Size)).get(Timestamp),
              CurrentRotation, 
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
          CurrentRotation, 
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
          Positions[Index].toPose2d(),
          Timestamps[Index],
          Deviation
        );
      });
    });
    Objects.ODOMETRY_LOCK.unlock();
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  private static synchronized Double discretize() {
    var DiscretizationTimestep = (0d);
    if (CurrentTime.equals((0d))) {
      DiscretizationTimestep = ((1d) / (50d));
    } else {
      var MeasuredTime = Logger.getRealTimestamp() / (1e6);
      DiscretizationTimestep = MeasuredTime - CurrentTime;
      CurrentTime = MeasuredTime;
    }
    return DiscretizationTimestep;
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
  }

  /**
   * Toggles between the possible states of orientation types
   */
  public static synchronized void toggleOrientationType() {
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
   * Provides implementation for creating repeatable pose alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this pose autonomously
   */
  public static Command alignPose(final Pose2d Source) {
    return SubsystemManager.pathfind(
      Source.transformBy(
      new Transform2d(
        GYROSCOPE.getYawRotation().getRadians() % 2 * Math.PI > Math.PI? -Measurements.ROBOT_RADIUS_METERS: Measurements.ROBOT_RADIUS_METERS,
        (0d),
        new Rotation2d())),
      (0d));
  }

  /**
   * Provides implementation for creating repeatable translation alignment
   * @param Source Pose to align the chassis to
   * @return Command configured to achieve this translation autonomously
   */
  public static Command alignTranslation(final Transform2d Source) {
    return SubsystemManager.pathfind(
      getPose().plus(
        new Transform2d(Source.getTranslation(),
                        Source.getRotation())
      ),
      (0d));
  }

  @Override
  public void configure(final Operator<Keybindings, Preferences> Profile) {
    CurrentOperator = Profile;
    DrivebaseSubsystem.getInstance().setDefaultCommand(
      new InstantCommand(() ->
      DrivebaseSubsystem.set(((Boolean) CurrentOperator.getPreference(Preferences.SQUARED_INPUT))?
        new Translation2d(
            square(MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.TRANSLATION_X_INPUT),
          (Double) CurrentOperator.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE))),
            square(MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.TRANSLATION_Y_INPUT),
          (Double) CurrentOperator.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE)))):
        new Translation2d(
          MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.TRANSLATION_X_INPUT),
        (Double) CurrentOperator.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE)),
          MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.TRANSLATION_Y_INPUT),
        (Double) CurrentOperator.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE))),
        ((Boolean) CurrentOperator.getPreference(Preferences.SQUARED_INPUT))?
        new Rotation2d(
            square(MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.ORIENTATION_T_INPUT),
          (Double) CurrentOperator.getPreference(Preferences.ORIENTATION_DEADZONE)))):
        new Rotation2d(
          (MathUtil.applyDeadband(-(Double) CurrentOperator.getPreference(Preferences.ORIENTATION_T_INPUT),
        (Double) CurrentOperator.getPreference(Preferences.ORIENTATION_DEADZONE))))),
        DrivebaseSubsystem.getInstance()
    ));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.ORIENTATION_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::toggleOrientationType,
          DrivebaseSubsystem.getInstance()
        )));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.MODULE_LOCKING_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::toggle,
          DrivebaseSubsystem.getInstance()
        )));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_SPEAKER)
      .onTrue(new InstantCommand(
        () -> {
          final var Rotation = GYROSCOPE.getYawRotation().getRadians();
          VisionSubsystem.getAprilTagPose(Rotation % 2 * Math.PI > Math.PI? (3): (7))
            .ifPresent((Pose) -> {
                alignPose(Pose.toPose2d())
              .onlyWhile(() -> CurrentOperator.getKeybinding(Keybindings.ORIENTATION_TOGGLE).getAsBoolean()).schedule();
            });
        },
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_AMP)
      .onTrue(new InstantCommand(
        () -> {
          VisionSubsystem.getAprilTagPose((9))
            .ifPresent((Pose) -> {
                alignPose(Pose.toPose2d())
              .onlyWhile(() -> CurrentOperator.getKeybinding(Keybindings.ORIENTATION_TOGGLE).getAsBoolean()).schedule();
            });
        },
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_OBJECT)
      .onTrue(new InstantCommand(
        () -> {
          VisionSubsystem.getOptimalTarget(CameraIdentifier.INTAKE_CAMERA)
            .ifPresent((Transformation) -> {
              alignTranslation(new Transform2d(Transformation.getTranslation().toTranslation2d(), Transformation.getRotation().toRotation2d()))
              .onlyWhile(() -> CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_OBJECT).getAsBoolean()).schedule();
            });
        },
        VisionSubsystem.getInstance(),
        DrivebaseSubsystem.getInstance()
      )));

    with(() ->
      CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_NEAREST)
      .onTrue(new InstantCommand(
        () -> {
          VisionSubsystem.getOptimalTarget(List.of(CameraIdentifier.SOURCE_CAMERA, CameraIdentifier.SPEAKER_FRONT_CAMERA, CameraIdentifier.SPEAKER_REAR_CAMERA))
            .ifPresent((Transformation) -> {
                alignTranslation(new Transform2d(Transformation.getTranslation().toTranslation2d(), Transformation.getRotation().toRotation2d()))
              .onlyWhile(() -> CurrentOperator.getKeybinding(Keybindings.ALIGNMENT_NEAREST).getAsBoolean()).schedule();
            });
        },
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


  /**
   * Applies squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  private static Double square(final Double Input) {
    return Math.copySign(Input * Input, Input);
  }

  /**
   * Toggles between the modules should go into a locking format when idle or not
   */
  public static synchronized void toggle() {
    ModuleLocked = !ModuleLocked;
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
      Logger.recordOutput(("Drivebase/Translation"), new Translation2d(Discrete.vxMetersPerSecond, Discrete.vyMetersPerSecond));
      Logger.recordOutput(("Drivebase/Rotation"), new Rotation2d(Discrete.omegaRadiansPerSecond));
      set(List.of(Reference));
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
        VisionSubsystem.getOptimalTarget(CameraIdentifier.INTAKE_CAMERA).ifPresentOrElse((Optimal) -> {
          set(new ChassisSpeeds(
            -Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
            -Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
            (-(Math.PI) + Optimal.getRotation().toRotation2d().getRadians() * (GYROSCOPE.getYawRotation().getRadians() % 2 * Math.PI > Math.PI? (-1): (1))) * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY
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
          Rotation.getRadians() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY
        ));
        break;
      case FIELD_ORIENTED:
        set(ChassisSpeeds.fromFieldRelativeSpeeds(
          -Translation.getX() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          -Translation.getY() * Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
          Rotation.getRadians() * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY,
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
    final var StateIterator = States.iterator();
    Logger.recordOutput(("Drivebase/Reference"), States.toArray(SwerveModuleState[]::new));
    Logger.recordOutput(("Drivebase/Optimized"),
      MODULES.stream().map((Module) ->
        Module.set(StateIterator.next())).toArray(SwerveModuleState[]::new));
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
    POSE_ESTIMATOR.resetPosition(GYROSCOPE.getYawRotation(),getModulePositions(),getPose());
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public Operator<Keybindings, Preferences> getOperator() {
    return CurrentOperator;
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
    return PathFlipped;
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