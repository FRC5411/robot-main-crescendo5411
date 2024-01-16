// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Objects;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.pathfinding.LocalADStarAK;
import org.robotalons.lib.motion.sensors.Gyroscope;

import java.io.Closeable;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
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
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class DrivebaseSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SwerveDriveKinematics KINEMATICS;
  private static final List<Module> MODULES;
  private static final Gyroscope GYROSCOPE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Rotation2d Odometry_Rotation;
  private static OrientationMode Control_Mode;
  private static DrivebaseSubsystem Instance;
  private static Pose2d Odometry_Pose;  
  private static Boolean Module_Locking;      
  private static Boolean Path_Flipped;  
  private static Double Current_Time;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {} static {
    Instance = new DrivebaseSubsystem();
    Odometry_Pose = new Pose2d();
    GYROSCOPE = new PigeonGyroscope(Constants.Measurements.PHOENIX_DRIVE);
    Control_Mode = OrientationMode.ROBOT_ORIENTED;
    Odometry_Rotation = GYROSCOPE.getYawRotation();
    Module_Locking = (false);
    Path_Flipped = (false); 
    Current_Time = Timer.getFPGATimestamp();
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
      Odometry_Pose
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
      () -> Path_Flipped,
      Instance);
      Pathfinding.setPathfinder(new LocalADStarAK());
      PathPlannerLogging.setLogActivePathCallback(
        (ActivePath) -> Logger.recordOutput(("Drivebase/Trajectory"), ActivePath.toArray(new Pose2d[0])));
      PathPlannerLogging.setLogTargetPoseCallback(
        (TargetPose) -> Logger.recordOutput(("Drivebase/Reference"), TargetPose));
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCK.lock();
    MODULES.forEach(Module::update);
    GYROSCOPE.update();    
    if (DriverStation.isDisabled()) {
      MODULES.forEach(Module::cease);
    }
    org.robotalons.crescendo.Constants.Subsystems.ROBOT_FIELD.setRobotPose(getPose());
    Objects.ODOMETRY_LOCK.unlock();
    AtomicInteger DeltaCount = new AtomicInteger(
      GYROSCOPE.getConnected()? 
        GYROSCOPE.getOdometryYawRotations().length: 
        Integer.MAX_VALUE
    );
    MODULES.forEach((Module) -> 
      DeltaCount.set(Math.min(DeltaCount.get(), Module.getPositionDeltas().size())
    ));
    IntStream.range((0), DeltaCount.get()).forEachOrdered((DeltaIndex) -> {
      SwerveModulePosition[] WheelDeltas = MODULES.stream().map(
        (Module) -> Module.getPositionDeltas().get(DeltaIndex)).toArray(SwerveModulePosition[]::new);
      var TwistDelta = KINEMATICS.toTwist2d(WheelDeltas);
      if(GYROSCOPE.getConnected()) {
        Rotation2d GyroRotationDelta = GYROSCOPE.getOdometryYawRotations()[DeltaIndex];
        TwistDelta = new Twist2d(TwistDelta.dx, TwistDelta.dy, GyroRotationDelta.minus(Odometry_Rotation).getRadians());
        Odometry_Rotation = GyroRotationDelta;
      }
      Odometry_Pose = Odometry_Pose.exp(TwistDelta);    
      var LocalizedTime = Timer.getFPGATimestamp();  
      POSE_ESTIMATOR.updateWithTime(
        LocalizedTime,
        Odometry_Rotation,
        WheelDeltas
      );
    });
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  @AutoLogOutput(key = "Drivebase/DiscretizationTimestamp")
  private static synchronized Double discretize() {
    var DiscretizationTimestep = (0.0);
    if (Current_Time.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Timer.getFPGATimestamp();
      DiscretizationTimestep = MeasuredTime - Current_Time;
      Current_Time = MeasuredTime;
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
    switch (Control_Mode) {
      case ROBOT_ORIENTED:
        Control_Mode = OrientationMode.FIELD_ORIENTED;
      case FIELD_ORIENTED:
        Control_Mode = OrientationMode.OBJECT_ORIENTED;
      case OBJECT_ORIENTED:
        Control_Mode = OrientationMode.ROBOT_ORIENTED;
    }
  }

  /**
   * Toggles between the if modules she go into a locking format when idle or not
   */
  public static synchronized void toggleModuleLocking() {
    Module_Locking = !Module_Locking;
  }

  /**
   * Toggles between is pathfinding should be flipped or not.
   */
  public static synchronized void togglePathFlipped() {
    Path_Flipped = !Path_Flipped;
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
    if (Demand.omegaRadiansPerSecond > 1e-6 && Demand.vxMetersPerSecond > 1e-6 && Demand.vyMetersPerSecond > 1e-6) {
      set();
    } else {
      var DiscretizedChassisSpeeds = ChassisSpeeds.discretize(Demand, discretize());
      var ReferenceStates = KINEMATICS.toSwerveModuleStates(DiscretizedChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
        ReferenceStates, 
        DiscretizedChassisSpeeds, 
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
        Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY);
      Logger.recordOutput(("Drivebase/Reference"), ReferenceStates);
      Logger.recordOutput(("Drivebase/Optimized"),
        IntStream.range((0), MODULES.size()).boxed().map(
          (Index) -> 
            MODULES.get(Index).set(ReferenceStates[Index]))
          .toArray(SwerveModuleState[]::new));
    }
  }

  /**
   * Drives the robot provided translation and rotational demands
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   * @param Mode        Type of demand being made
   */
  public static synchronized void set(final Translation2d Translation, final Rotation2d Rotation, final OrientationMode Mode) {
    switch(Mode) {
      case OBJECT_ORIENTED:
        //TODO: AUTOMATION TEAM
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
    if(Module_Locking) {
      KINEMATICS.resetHeadings(MODULES.stream().map((Module) -> 
        Module.getObserved().angle
      ).toArray(Rotation2d[]::new));  
    }
    set(new ChassisSpeeds());
  }

  /**
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static synchronized void set(final Pose2d Pose) {
    Odometry_Pose = Pose;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
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