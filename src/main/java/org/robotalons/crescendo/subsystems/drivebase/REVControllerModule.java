// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
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
  private final List<Double> TIMESTAMPS;
  private final RelativeEncoder ROTATIONAL_ENCODER;
  private final RelativeEncoder LINEAR_ENCODER;
  private final ModuleConstants CONSTANTS;
  private final Queue<Double> ROTATIONAL_QUEUE;    
  private final Queue<Double> TIMESTAMP_QUEUE;
  private final Queue<Double> LINEAR_QUEUE;
  private final Lock ODOMETRY_LOCK;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private ReferenceType ReferenceMode;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * REV Controller Module Constructor
   * @param Constants Constants of new module instance
   */
  public REVControllerModule(final ModuleConstants Constants) {
    super(Constants);

    ReferenceMode = ReferenceType.STATE_CONTROL;
    CONSTANTS = Constants;

    CONSTANTS.LINEAR_CONTROLLER.restoreFactoryDefaults();
    CONSTANTS.ROTATIONAL_CONTROLLER.restoreFactoryDefaults();

    CONSTANTS.LINEAR_CONTROLLER.setInverted(CONSTANTS.LINEAR_INVERTED);
    CONSTANTS.ROTATIONAL_CONTROLLER.setInverted(CONSTANTS.ROTATIONAL_INVERTED);

    LINEAR_ENCODER = CONSTANTS.LINEAR_CONTROLLER.getEncoder();
    ROTATIONAL_ENCODER = CONSTANTS.ROTATIONAL_CONTROLLER.getEncoder();
      
    RotationalAbsoluteOffset = CONSTANTS.ROTATIONAL_ENCODER_OFFSET;

    CONSTANTS.LINEAR_CONTROLLER.setCANTimeout((250));
    CONSTANTS.ROTATIONAL_CONTROLLER.setCANTimeout((250));
    CONSTANTS.ROTATIONAL_CONTROLLER.setInverted((true));

    CONSTANTS.LINEAR_CONTROLLER.setPeriodicFramePeriod(
      PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
    CONSTANTS.ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));

    CONSTANTS.LINEAR_CONTROLLER.setSmartCurrentLimit((40));
    CONSTANTS.ROTATIONAL_CONTROLLER.setSmartCurrentLimit((30));
    
    CONSTANTS.LINEAR_CONTROLLER.enableVoltageCompensation((12d));
    CONSTANTS.ROTATIONAL_CONTROLLER.enableVoltageCompensation((12d));

    LINEAR_ENCODER.setPosition((0d));
    LINEAR_ENCODER.setAverageDepth((2));
    LINEAR_ENCODER.setMeasurementPeriod((10));
    
    CONSTANTS.LINEAR_CONTROLLER.setIdleMode(IdleMode.kBrake);
    CONSTANTS.ROTATIONAL_CONTROLLER.setIdleMode(IdleMode.kCoast);

    ROTATIONAL_ENCODER.setPosition(-RotationalAbsoluteOffset.plus(Rotation2d.fromRotations(CONSTANTS.ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble())).getRotations());
    ROTATIONAL_ENCODER.setAverageDepth((2));
    ROTATIONAL_ENCODER.setMeasurementPeriod((10));

    CONSTANTS.LINEAR_CONTROLLER.setCANTimeout((0));
    CONSTANTS.ROTATIONAL_CONTROLLER.setCANTimeout((0));

    CONSTANTS.ROTATIONAL_CONTROLLER_PID.enableContinuousInput(-Math.PI, Math.PI);

    CONSTANTS.LINEAR_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
    CONSTANTS.ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
    ODOMETRY_LOCK = new ReentrantLock();    
    LINEAR_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(LINEAR_ENCODER::getPosition);
    ROTATIONAL_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(ROTATIONAL_ENCODER::getPosition); 
    TIMESTAMP_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.timestamp();
    CONSTANTS.ROTATIONAL_CONTROLLER.burnFlash();
    CONSTANTS.LINEAR_CONTROLLER.burnFlash();
    TIMESTAMPS = new ArrayList<>();
    DELTAS = new ArrayList<>();
    reset();
    Reference = new SwerveModuleState();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
  public static final class ModuleConstants extends Constants {
    public SimpleMotorFeedforward LINEAR_CONTROLLER_FEEDFORWARD;
    public PIDController ROTATIONAL_CONTROLLER_PID;
    public Rotation2d ROTATIONAL_ENCODER_OFFSET;
    public PIDController LINEAR_CONTROLLER_PID;
    public CANSparkMax ROTATIONAL_CONTROLLER; 
    public CANSparkMax LINEAR_CONTROLLER;
    public CANcoder ABSOLUTE_ENCODER;
    public Boolean ROTATIONAL_INVERTED;
    public Boolean LINEAR_INVERTED;
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
      Rotation2d.fromRotations(CONSTANTS.ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble()).minus(RotationalAbsoluteOffset);
    Status.RotationalRelativePosition =
      Rotation2d.fromRotations(ROTATIONAL_ENCODER.getPosition() / CONSTANTS.ROTATION_GEAR_RATIO);
    Status.RotationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_ENCODER.getVelocity()) / CONSTANTS.ROTATION_GEAR_RATIO;
    Status.RotationalAppliedVoltage = 
      CONSTANTS.ROTATIONAL_CONTROLLER.getAppliedOutput() * CONSTANTS.ROTATIONAL_CONTROLLER.getBusVoltage();
    Status.RotationalAppliedAmperage = 
      new double[] {CONSTANTS.ROTATIONAL_CONTROLLER.getOutputCurrent()};
    Status.OdometryTimestamps = TIMESTAMP_QUEUE.stream().mapToDouble(Double::valueOf).toArray();
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
    TIMESTAMP_QUEUE.clear();
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(CONSTANTS.NUMBER) + ')', Status);
  }

  @Override
  public synchronized void periodic() {
    ODOMETRY_LOCK.lock();
    update();
    if (RotationalRelativeOffset == (null) && Status.RotationalAbsolutePosition.getRadians() != (0d)) {
      RotationalRelativeOffset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
    }
    switch(ReferenceMode) {
      case STATE_CONTROL:
        if(Reference != (null)) {
          if (Reference.angle != (null)) {
            setRotationVoltage(CONSTANTS.ROTATIONAL_CONTROLLER_PID.calculate(getRelativeRotation().getRadians(),Reference.angle.getRadians()));
          }
          var AdjustReferenceVelocity = (Reference.speedMetersPerSecond * Math.cos(CONSTANTS.ROTATIONAL_CONTROLLER_PID.getPositionError())) / CONSTANTS.WHEEL_RADIUS_METERS;
          setLinearVoltage(     -(CONSTANTS.LINEAR_CONTROLLER_PID.calculate(AdjustReferenceVelocity))
                                                              +
            (CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD.calculate(Status.LinearVelocityRadiansSecond, AdjustReferenceVelocity)) 
                                                              * 
                                            ((CONSTANTS.LINEAR_INVERTED)? (-1): (1)));          
        }
        break;
      case DISABLED:
        cease();
        break;
      case CLOSED:
        close();
        break;
    }
    ODOMETRY_LOCK.unlock();  
    DELTAS.clear();
    IntStream.range((0), Status.OdometryTimestamps.length).forEach((Index) -> {
      final var Position = Status.OdometryLinearPositionsRadians[Index] * CONSTANTS.WHEEL_RADIUS_METERS;
      final var Rotation = Status.OdometryAzimuthPositions[Index].plus(
        RotationalRelativeOffset != null? RotationalRelativeOffset: new Rotation2d());
      DELTAS.add(new SwerveModulePosition(Position, Rotation));
    });
  }

  /**
   * Zeroes the azimuth relatively offset from the position of the absolute encoders.
   */
  public synchronized void reset() {
    update();
    RotationalAbsoluteOffset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  @Override
  public SwerveModuleState set(final SwerveModuleState Reference) {
    this.Reference = SwerveModuleState.optimize(Reference, getRelativeRotation());
    return this.Reference;
  }

  @Override
  public void set(final ReferenceType Mode) {
    ReferenceMode = Mode;
  }

  /**
   * Mutator for the Rotational Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setRotationVoltage(final Double Demand) {
    CONSTANTS.ROTATIONAL_CONTROLLER.setVoltage(MathUtil.clamp(Demand, (-12d), (12d)));
  }

  /**
   * Mutator for the Linear Controller's current rotational voltage supply
   * @param Demand Demand of Voltage, relative to battery
   */
  public void setLinearVoltage(final Double Demand) {
    CONSTANTS.LINEAR_CONTROLLER.setVoltage(MathUtil.clamp(Demand, (-12d), (12d)));
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    return DELTAS;
  }

  public List<Double> getPositionTimestamps() {
    return TIMESTAMPS;
  }

  public Rotation2d getRelativeRotation() {
    return Status.RotationalRelativePosition.plus(RotationalRelativeOffset);
  }

  public Rotation2d getAbsoluteRotation() {
    return Status.RotationalAbsolutePosition.plus(RotationalAbsoluteOffset);
  }

  public Double getLinearPosition() {
    return Status.LinearPositionRadians * CONSTANTS.WHEEL_RADIUS_METERS;
  }
}