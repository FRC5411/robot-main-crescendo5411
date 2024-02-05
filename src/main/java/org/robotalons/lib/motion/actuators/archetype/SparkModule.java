// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators.archetype;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
// --------------------------------------------------------------[Spark Module]-------------------------------------------------------------//
/**
 *
 *
 * <h1>REVControllerModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes REV Controllers (SparkMax) as hardware.</p>
 * 
 * @see Module
 */
public class SparkModule<Controller extends CANSparkMax> extends Module {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Controller TRANSLATIONAL_CONTROLLER;
  private final SparkMaxPIDController TRANSLATIONAL_PID;
  private final SimpleMotorFeedforward TRANSLATIONAL_FF;
  private final RelativeEncoder TRANSLATIONAL_ENCODER;
  private final Queue<Double> TRANSLATIONAL_VELOCITY_QUEUE;

  private final Controller ROTATIONAL_CONTROLLER;
  private final SparkMaxPIDController ROTATIONAL_PID;
  private final RelativeEncoder ROTATIONAL_ENCODER;
  private final Queue<Double> ROTATIONAL_POSITION_QUEUE;   

  private final CANcoder ABSOLUTE_ENCODER;
  private final Queue<Double> TIMESTAMP_QUEUE;
  private final List<SwerveModulePosition> DELTAS;
  private final List<Double> TIMESTAMPS;
  private final Lock ODOMETRY_LOCK;

  private final ModuleConfiguration<Controller> MODULE_CONSTANTS;
  // -----------------------------------------------------------[Constructor(s)]------------------------------------------------------------//
  /**
   * Spark Module Constructor
   * @param Constants Constant values to construct a new module from
   */
  public SparkModule(final ModuleConfiguration<Controller> Constants) {
    super(Constants);
    MODULE_CONSTANTS = Constants;
    TRANSLATIONAL_CONTROLLER = MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER;
    TRANSLATIONAL_PID = TRANSLATIONAL_CONTROLLER.getPIDController();
    TRANSLATIONAL_FF = new SimpleMotorFeedforward(
      MODULE_CONSTANTS.TRANSLATIONAL_KS_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KV_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KA_GAIN);
    TRANSLATIONAL_ENCODER = TRANSLATIONAL_CONTROLLER.getEncoder();
    ROTATIONAL_CONTROLLER = MODULE_CONSTANTS.ROTATIONAL_CONTROLLER;
    ROTATIONAL_PID = ROTATIONAL_CONTROLLER.getPIDController();
    ROTATIONAL_ENCODER = ROTATIONAL_CONTROLLER.getEncoder();
    ABSOLUTE_ENCODER = new CANcoder(MODULE_CONSTANTS.ABSOLUTE_ENCODER_PORT);
    
    TRANSLATIONAL_VELOCITY_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(TRANSLATIONAL_ENCODER::getVelocity);
    ROTATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(ROTATIONAL_ENCODER::getPosition);
    TIMESTAMP_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.timestamp();
    ODOMETRY_LOCK = new ReentrantLock();

    TIMESTAMPS = new ArrayList<>();
    DELTAS = new ArrayList<>();

    RotationalAbsoluteOffset = MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET;

    configure();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Force configures this module's hardware to standard SparkModule specifications.
   */
  private synchronized void configure() {
    cease();
    ODOMETRY_LOCK.lock();

    TRANSLATIONAL_CONTROLLER.restoreFactoryDefaults();
    ROTATIONAL_CONTROLLER.restoreFactoryDefaults();

    TRANSLATIONAL_CONTROLLER.setCANTimeout((250));
    ROTATIONAL_CONTROLLER.setCANTimeout((250));

    ROTATIONAL_CONTROLLER.setInverted(MODULE_CONSTANTS.ROTATIONAL_INVERTED);
    TRANSLATIONAL_CONTROLLER.setInverted(MODULE_CONSTANTS.TRANSLATIONAL_INVERTED);

    TRANSLATIONAL_CONTROLLER.setIdleMode(IdleMode.kCoast);
    ROTATIONAL_CONTROLLER.setIdleMode(IdleMode.kBrake);

    TRANSLATIONAL_CONTROLLER.setSmartCurrentLimit((40));
    ROTATIONAL_CONTROLLER.setSmartCurrentLimit((30));
    TRANSLATIONAL_CONTROLLER.enableVoltageCompensation((10d));
    ROTATIONAL_CONTROLLER.enableVoltageCompensation((10d));

    TRANSLATIONAL_ENCODER.setPosition((0d));
    TRANSLATIONAL_ENCODER.setMeasurementPeriod((10));
    TRANSLATIONAL_ENCODER.setAverageDepth((2));

    TRANSLATIONAL_PID.setP(MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kP);
    TRANSLATIONAL_PID.setI(MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kI);
    TRANSLATIONAL_PID.setD(MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kD);

    ROTATIONAL_PID.setP(MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kP);
    ROTATIONAL_PID.setI(MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kI);
    ROTATIONAL_PID.setD(MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kD);

    ROTATIONAL_PID.setPositionPIDWrappingEnabled((true));
    ROTATIONAL_PID.setPositionPIDWrappingMaxInput(Math.PI);
    ROTATIONAL_PID.setPositionPIDWrappingMinInput(-Math.PI);
    ROTATIONAL_PID.setOutputRange(
      -(1), 
       (1));
    ROTATIONAL_PID.setFeedbackDevice(ROTATIONAL_ENCODER);
    
    TRANSLATIONAL_PID.setOutputRange(
      -(1), 
       (1));
    TRANSLATIONAL_PID.setFeedbackDevice(TRANSLATIONAL_ENCODER);

    ROTATIONAL_ENCODER.setPosition(
      (RobotBase.isReal())?
      (-RotationalAbsoluteOffset
              .plus(
        Rotation2d.fromRotations(
          ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble())
        ).getRotations()
      ):
      (0d)
    );

    ROTATIONAL_ENCODER.setMeasurementPeriod((10));
    ROTATIONAL_ENCODER.setAverageDepth((2));

    TRANSLATIONAL_CONTROLLER.setPeriodicFramePeriod(
      PeriodicFrame.kStatus2,
      (int) ((1000d) / MODULE_CONSTANTS.STATUS_PROVIDER.getFrequency()
    ));
    ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
      PeriodicFrame.kStatus2,
      (int) ((1000d) /  MODULE_CONSTANTS.STATUS_PROVIDER.getFrequency()
    ));

    TRANSLATIONAL_CONTROLLER.setCANTimeout((0));
    ROTATIONAL_CONTROLLER.setCANTimeout((0));

    TRANSLATIONAL_CONTROLLER.burnFlash();
    ROTATIONAL_CONTROLLER.burnFlash();

    ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void close() {
    ReferenceMode = ReferenceType.CLOSED;
    TRANSLATIONAL_CONTROLLER.close();
    ROTATIONAL_CONTROLLER.close();
    ABSOLUTE_ENCODER.close();
    TRANSLATIONAL_VELOCITY_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    TIMESTAMPS.clear();
    DELTAS.clear();
  }

  @Override
  public synchronized void cease() {
    TRANSLATIONAL_CONTROLLER.stopMotor();
    ROTATIONAL_CONTROLLER.stopMotor();
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
            ROTATIONAL_PID.setReference(Reference.angle.getRadians(), ControlType.kPosition);
          }
          Reference.speedMetersPerSecond *= Math.cos(Reference.angle.minus(getRelativeRotation()).getRadians());
          TRANSLATIONAL_PID.setReference(Reference.speedMetersPerSecond, ControlType.kVelocity, (0), 
            TRANSLATIONAL_FF.calculate(Reference.speedMetersPerSecond), ArbFFUnits.kVoltage);
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
      final var Position = Status.OdometryTranslationalPositionsRadians[Index] * CONSTANTS.WHEEL_RADIUS_METERS;
      final var Rotation = Status.OdometryRotationalPositions[Index].plus(
        RotationalRelativeOffset != null? RotationalRelativeOffset: new Rotation2d());
      DELTAS.add(new SwerveModulePosition(Position, Rotation));
    });
  }
  
  /**
   * Forces an update of the AutoLogged values to AdvantageKit, logged with the corresponding module number
   */
  @Override
  public synchronized void update() {
    Status.TranslationalPositionRadians =
        Units.rotationsToRadians(TRANSLATIONAL_ENCODER.getPosition()) / CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
    Status.TranslationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(TRANSLATIONAL_ENCODER.getVelocity()) / CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
    Status.TranslationalAppliedVoltage = 
      TRANSLATIONAL_CONTROLLER.getAppliedOutput() * TRANSLATIONAL_CONTROLLER.getBusVoltage();
    Status.TranslationalCurrentAmperage = TRANSLATIONAL_CONTROLLER.getOutputCurrent();
    Status.RotationalAbsolutePosition = 
      Rotation2d.fromRotations(ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble()).minus(RotationalAbsoluteOffset);
    Status.RotationalRelativePosition =
      Rotation2d.fromRotations(ROTATIONAL_ENCODER.getPosition() / CONSTANTS.ROTATIONAL_GEAR_RATIO);
    Status.RotationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_ENCODER.getVelocity()) / CONSTANTS.ROTATIONAL_GEAR_RATIO;
    Status.RotationalAppliedVoltage = 
      ROTATIONAL_CONTROLLER.getAppliedOutput() * ROTATIONAL_CONTROLLER.getBusVoltage();
    Status.RotationalAppliedAmperage = ROTATIONAL_CONTROLLER.getOutputCurrent();
    Status.OdometryTimestamps = TIMESTAMP_QUEUE.stream().mapToDouble(Double::valueOf).toArray();
    Status.OdometryTranslationalPositionsRadians =
      TRANSLATIONAL_VELOCITY_QUEUE.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.ROTATIONAL_GEAR_RATIO)
        .toArray();
    Status.OdometryRotationalPositions =
      ROTATIONAL_POSITION_QUEUE.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / CONSTANTS.TRANSLATIONAL_GEAR_RATIO))
        .toArray(Rotation2d[]::new);
    TRANSLATIONAL_VELOCITY_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(CONSTANTS.NUMBER) + ')', Status);
  }

  /**
   * Zeroes the rotational relative to offset from the position of the absolute encoders.
   */
  @Override
  public synchronized void reset() {
    update();
    ROTATIONAL_ENCODER.setPosition(
      (RobotBase.isReal())?
      (-RotationalAbsoluteOffset
              .plus(
        Rotation2d.fromRotations(
          ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble())
        ).getRotations()
      ):
      (0d)
    );
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    return DELTAS;
  }

  @Override
  public List<Double> getPositionTimestamps() {
    return TIMESTAMPS;
  }
}
