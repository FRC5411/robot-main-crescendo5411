// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;

// ----------------------------------------------------------[REV Controller Module]--------------------------------------------------------//
/**
 *
 *
 * <h1>REVControllerModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes REV Controllers (SparkMax) as hardware.</p>
 * 
 * @see Module
 * @see DrivebaseSubsystem
 */
public final class REVControllerModule extends Module {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final List<SwerveModulePosition> DELTAS;
  private final ModuleConstants CONSTANTS;
  private final RelativeEncoder LINEAR_ENCODER;
  private final RelativeEncoder ROTATIONAL_ENCODER;
  private final Queue<Double> LINEAR_QUEUE;
  private final Queue<Double> ROTATIONAL_QUEUE;    
  private final Lock ODOMETRY_LOCK;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private ReferenceType ReferenceMode;
  private Double CurrentPosition;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Controller Module Constructor
   * @param Constants Constants of new module instance
   */
  public REVControllerModule(final ModuleConstants Constants) {
    super(Constants);
    ReferenceMode = ReferenceType.STATE_CONTROL;
    CurrentPosition = (0.0);
    CONSTANTS = Constants;
    
    CONSTANTS.LINEAR_CONTROLLER.restoreFactoryDefaults();
    CONSTANTS.ROTATIONAL_CONTROLLER.restoreFactoryDefaults();
    LINEAR_ENCODER = CONSTANTS.LINEAR_CONTROLLER.getEncoder();
    ROTATIONAL_ENCODER = CONSTANTS.ROTATIONAL_CONTROLLER.getEncoder();
    CONSTANTS.LINEAR_CONTROLLER.setCANTimeout((250));
    CONSTANTS.ROTATIONAL_CONTROLLER.setCANTimeout((250));

    CONSTANTS.LINEAR_CONTROLLER.setSmartCurrentLimit((40));
    CONSTANTS.ROTATIONAL_CONTROLLER.setSmartCurrentLimit((30));
    CONSTANTS.LINEAR_CONTROLLER.enableVoltageCompensation((12.0));
    CONSTANTS.ROTATIONAL_CONTROLLER.enableVoltageCompensation((12.0));

    LINEAR_ENCODER.setPosition((0.0));
    LINEAR_ENCODER.setMeasurementPeriod((10));
    LINEAR_ENCODER.setAverageDepth((2));

    ROTATIONAL_ENCODER.setPosition((0.0));
    ROTATIONAL_ENCODER.setMeasurementPeriod((10));
    ROTATIONAL_ENCODER.setAverageDepth((2));

    CONSTANTS.LINEAR_CONTROLLER.setCANTimeout((0));
    CONSTANTS.ROTATIONAL_CONTROLLER.setCANTimeout((0));

    CONSTANTS.LINEAR_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
    CONSTANTS.ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
    ODOMETRY_LOCK = new ReentrantLock();    
    LINEAR_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(LINEAR_ENCODER::getPosition);
    ROTATIONAL_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(ROTATIONAL_ENCODER::getPosition); 
    CONSTANTS.LINEAR_CONTROLLER.burnFlash();
    CONSTANTS.ROTATIONAL_CONTROLLER.burnFlash();
    DELTAS = new ArrayList<>();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class ModuleConstants extends Constants {
    public SimpleMotorFeedforward LINEAR_CONTROLLER_FEEDFORWARD;
    public PIDController ROTATIONAL_CONTROLLER_PID;
    public PIDController LINEAR_CONTROLLER_PID;
    public CANSparkMax ROTATIONAL_CONTROLLER; 
    public CANSparkMax LINEAR_CONTROLLER;
    public WPI_CANCoder ABSOLUTE_ENCODER;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  @Override
  public void close() {
    CONSTANTS.LINEAR_CONTROLLER.disable();
    CONSTANTS.ROTATIONAL_CONTROLLER.disable();
    LINEAR_QUEUE.clear();
    ROTATIONAL_QUEUE.clear();
  }

  @Override
  public void cease() {
    CONSTANTS.LINEAR_CONTROLLER.set((0d));
    CONSTANTS.ROTATIONAL_CONTROLLER.set((0d));
  }

  @Override
  public void update() {
    Status.LinearPositionRadians =
        Units.rotationsToRadians(LINEAR_ENCODER.getPosition()) / CONSTANTS.LINEAR_GEAR_RATIO;
    Status.LinearVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(LINEAR_ENCODER.getVelocity()) / CONSTANTS.LINEAR_GEAR_RATIO;
    Status.LinearAppliedVoltage = 
      CONSTANTS.LINEAR_CONTROLLER.getAppliedOutput() * CONSTANTS.LINEAR_CONTROLLER.getBusVoltage();
    Status.LinearCurrentAmperage = 
      new double[] {CONSTANTS.LINEAR_CONTROLLER.getOutputCurrent()};
    Status.RotationalAbsolutePosition = 
      Rotation2d.fromDegrees(CONSTANTS.ABSOLUTE_ENCODER.getAbsolutePosition());
    Status.RotationalRelativePosition =
        Rotation2d.fromRotations(ROTATIONAL_ENCODER.getPosition() / CONSTANTS.ROTATION_GEAR_RATIO);
    Status.RotationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_ENCODER.getVelocity()) / CONSTANTS.ROTATION_GEAR_RATIO;
    Status.RotationalAppliedVoltage = 
      CONSTANTS.ROTATIONAL_CONTROLLER.getAppliedOutput() * CONSTANTS.ROTATIONAL_CONTROLLER.getBusVoltage();
    Status.RotationalAppliedAmperage = 
      new double[] {CONSTANTS.ROTATIONAL_CONTROLLER.getOutputCurrent()};
    Status.OdometryLinearPositionsRadians =
      LINEAR_QUEUE.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.ROTATION_GEAR_RATIO)
        .toArray();
    Status.OdometryAzimuthPositions =
      ROTATIONAL_QUEUE.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / CONSTANTS.LINEAR_GEAR_RATIO))
        .toArray(Rotation2d[]::new);
    LINEAR_QUEUE.clear();
    ROTATIONAL_QUEUE.clear();
  }

  @Override
  public void periodic() {
    ODOMETRY_LOCK.lock();
    switch(ReferenceMode) {
      case STATE_CONTROL:
        if (Objects.isNull(Azimuth_Offset) && Status.RotationalAbsolutePosition.getRadians() != (0d)) {
          Azimuth_Offset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
        }
        if (!Objects.isNull(Reference.angle)) {
          setRotationVoltage(CONSTANTS.ROTATIONAL_CONTROLLER_PID.calculate(getRelativeRotation().getRadians(),Reference.angle.getRadians()));
          if(!Objects.isNull(Reference.speedMetersPerSecond)) {
            var AdjustReferenceSpeed = Reference.speedMetersPerSecond * Math.cos(CONSTANTS.ROTATIONAL_CONTROLLER_PID.getPositionError()) / CONSTANTS.WHEEL_RADIUS_METERS;
            setLinearVoltage(
              (CONSTANTS.LINEAR_CONTROLLER_PID.calculate(AdjustReferenceSpeed))
                                    +
              (CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD.calculate(Status.LinearVelocityRadiansSecond, AdjustReferenceSpeed)));
          }
        }      
        break;
      case DISABLED:
        cease();
      case CLOSED:
        close();
        break;
    }
    DELTAS.clear();
    IntStream.range((0), Math.min(Status.OdometryLinearPositionsRadians.length, Status.OdometryAzimuthPositions.length)).forEach((Index) -> {
      DELTAS.add(new SwerveModulePosition((getLinearPosition() - CurrentPosition), getRelativeRotation()));
    });
    ODOMETRY_LOCK.unlock();    
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  @Override
  public SwerveModuleState set(SwerveModuleState Reference) {
    this.Reference = SwerveModuleState.optimize(Reference, getAbsoluteRotation());
    return this.Reference;
  }

  @Override
  public void set(ReferenceType Mode) {
    ReferenceMode = Mode;
  }

  /**
   * Mutator for the Rotational Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setRotationVoltage(final Double Demand) {
    CONSTANTS.ROTATIONAL_CONTROLLER.setVoltage(Demand);

  }

  /**
   * Mutator for the Linear Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setLinearVoltage(final Double Demand) {
    CONSTANTS.LINEAR_CONTROLLER.setVoltage(Demand);
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    return DELTAS;
  }

  public Rotation2d getRelativeRotation() {
    return (Objects.isNull(Azimuth_Offset))? (new Rotation2d()): (Status.RotationalRelativePosition.plus(Azimuth_Offset));
  }

  public Rotation2d getAbsoluteRotation() {
    return Rotation2d.fromDegrees(CONSTANTS.ABSOLUTE_ENCODER.getAbsolutePosition());
  }

  public Double getLinearPosition() {
    return Status.LinearPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS;
  }
}
