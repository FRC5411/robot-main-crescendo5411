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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Objects;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.sensors.Gyroscope;
import org.robotalons.lib.utilities.PilotProfile;

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
      new Pose2d() //TODO: AUTOMATION TEAM (DEFAULT FROM CAMERA STATES)
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
   * Resets the modules are pose estimation
   */
  public synchronized void reset() {
    MODULES.forEach(Module::reset);
    KINEMATICS.resetHeadings(MODULES.stream().map((Module) -> 
      Module.getObserved().angle
    ).toArray(Rotation2d[]::new));
    POSE_ESTIMATOR.resetPosition(
      GYROSCOPE.getYawRotation(),
      getModulePositions(),
      new Pose2d()
    );
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
        CurrentMode = OrientationMode.FIELD_ORIENTED;
      case FIELD_ORIENTED:
        CurrentMode = OrientationMode.OBJECT_ORIENTED;
      case OBJECT_ORIENTED:
        CurrentMode = OrientationMode.ROBOT_ORIENTED;
    }
  }

  @Override
  public void configure(final PilotProfile Pilot) {
    CurrentPilot = Pilot;
    DrivebaseSubsystem.getInstance().setDefaultCommand(
      new InstantCommand(() ->
      DrivebaseSubsystem.set(((Boolean) CurrentPilot.getPreference(Preferences.SQUARED_INPUT))?
        new Translation2d(
            applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE))),
            applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE)))):
          new Translation2d(
            MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_X_DEADZONE)),
            MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.TRANSLATIONAL_Y_DEADZONE))),
        ((Boolean) CurrentPilot.getPreference(Preferences.SQUARED_INPUT))?
        new Rotation2d(
            applySquared(MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.ORIENTATION_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.ORIENTATION_DEADZONE)))):
          new Rotation2d(
            (MathUtil.applyDeadband(-(Double) CurrentPilot.getPreference(Preferences.ORIENTATION_INPUT),
          (Double) CurrentPilot.getPreference(Preferences.ORIENTATION_DEADZONE))))), 
        DrivebaseSubsystem.getInstance()
    ));
    try {
      CurrentPilot.getKeybinding(Keybindings.ORIENTATION_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::toggleOrientationType,
          DrivebaseSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.MODULE_LOCKING_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::toggleModuleLocking,
          DrivebaseSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.PATHFINDING_FLIP_TOGGLE)
        .onTrue(new InstantCommand(
          DrivebaseSubsystem::togglePathFlipped,
          DrivebaseSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
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
  /**
   * Describes a robot's current mode of orientation
   */
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
        //TODO: AUTOMATION TEAM (OBJECT ORIENTATION DRIVEBASE)   
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
  /**
   * Provides the current pilot of the drivebase
   * @return Pilot of this subsystem
   */
  @Override
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
   * Provides the kinematics chassis speeds of the current module measurements
   * @return Drivebase kinematics objects
   */
  public static ChassisSpeeds getSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleMeasurements());
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