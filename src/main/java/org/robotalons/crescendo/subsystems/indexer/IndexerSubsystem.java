// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Closeable;

import org.robotalons.crescendo.subsystems.indexer.Constants.Measurements;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;

import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

// ------------------------------------------------------------[Indexer Subsystem]---------------------------------------------------------- //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes to and from the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */

public class IndexerSubsystem extends SubsystemBase implements Closeable {
    // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
    private final static CANSparkMax INDEXER_INTAKE_MOTOR;
    private final static CANSparkMax INDEXER_CANNON_MOTOR;
    private final static CANSparkMax[] MOTORS;
    
    private static final Encoder INDEXER_INTAKE_MOTOR_ENCODER;
    private static final Encoder INDEXER_CANNON_MOTOR_ENCODER;
    private static final Encoder[] ENCODERS;
    
    private static final SparkMaxPIDController INDEXER_INTAKE_MOTOR_PID;
    private static final SparkMaxPIDController INDEXER_CANNON_MOTOR_PID;

    private final static DigitalInput INDEXER_INTAKE_RECEIVER;
    private final static DigitalInput INDEXER_CANNON_RECEIVER;

    // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
    private static IndexerSubsystem Instance;
    private static Boolean HasNote;

    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /**
     * Indexer Subsystem Constructor
     */
    private IndexerSubsystem() {} static {
        Instance = new IndexerSubsystem();
        HasNote = (false);

        INDEXER_INTAKE_MOTOR = new CANSparkMax(Constants.Ports.INDEXER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        INDEXER_CANNON_MOTOR = new CANSparkMax(Constants.Ports.INDEXER_CANNON_MOTOR_ID, MotorType.kBrushless);
        MOTORS = new CANSparkMax[]{INDEXER_INTAKE_MOTOR, INDEXER_CANNON_MOTOR};

        INDEXER_INTAKE_MOTOR_ENCODER = new Encoder(Measurements.K_INDEXER_INTAKE_MOTOR_ENCODER_CHANNELA, Measurements.K_INDEXER_INTAKE_MOTOR_ENCODER_CHANNELB);
        INDEXER_CANNON_MOTOR_ENCODER = new Encoder(Measurements.K_INDEXER_CANNON_MOTOR_ENCODER_CHANNELA, Measurements.K_INDEXER_CANNON_MOTOR_ENCODER_CHANNELB);
        ENCODERS = new Encoder[]{INDEXER_INTAKE_MOTOR_ENCODER, INDEXER_CANNON_MOTOR_ENCODER};

        INDEXER_INTAKE_MOTOR_PID = INDEXER_INTAKE_MOTOR.getPIDController();
        INDEXER_CANNON_MOTOR_PID = INDEXER_CANNON_MOTOR.getPIDController();

        configPID();
        config(INDEXER_INTAKE_MOTOR);
        config(INDEXER_CANNON_MOTOR);

        INDEXER_INTAKE_RECEIVER = new DigitalInput(Constants.Ports.INDEXER_INTAKE_RECEIVER_ID);
        INDEXER_CANNON_RECEIVER = new DigitalInput(Constants.Ports.INDEXER_CANNON_RECEIVER_ID);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        Constants.Objects.ODOMETRY_LOCKER.lock();
        HasNote = !(INDEXER_INTAKE_RECEIVER.get()|| INDEXER_CANNON_RECEIVER.get());
        Constants.Objects.ODOMETRY_LOCKER.lock();
    }

    public synchronized static void configPID() {
        INDEXER_INTAKE_MOTOR_PID.setFeedbackDevice((MotorFeedbackSensor) INDEXER_INTAKE_MOTOR_ENCODER);
        INDEXER_INTAKE_MOTOR_PID.setP(Measurements.INDEXER_INTAKE_MOTOR_P);
        INDEXER_INTAKE_MOTOR_PID.setI(Measurements.INDEXER_INTAKE_MOTOR_I);
        INDEXER_INTAKE_MOTOR_PID.setD(Measurements.INDEXER_INTAKE_MOTOR_D);

        INDEXER_CANNON_MOTOR_PID.setFeedbackDevice((MotorFeedbackSensor) INDEXER_CANNON_MOTOR_ENCODER);
        INDEXER_CANNON_MOTOR_PID.setP(Measurements.INDEXER_CANNON_MOTOR_P);
        INDEXER_CANNON_MOTOR_PID.setI(Measurements.INDEXER_CANNON_MOTOR_I);
        INDEXER_CANNON_MOTOR_PID.setD(Measurements.INDEXER_CANNON_MOTOR_D);
    }

    public synchronized static void config(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kCoast);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
    
        motor.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);
    }
    
    /**
     * Closes this instance and all held resources immediately 
     */
    public synchronized void close() {
        INDEXER_INTAKE_MOTOR.close();
        INDEXER_CANNON_MOTOR.close();
    }
    
    // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
    /**
     * Describes the relationship of a relevant game piece to the position its going to be sent to.
     */
    public enum Direction {
        FORWARD_CANNON,     //Indexer --> Cannon   (called by INTAKE system after completing a pickup)
        FORWARD_INDEXER,    //Intake ---> Indexer  (called by CANNON system in order to put a Note in firing position)
        BACKWARD_INDEXER,   //Indexer --> Intake   (called manually (e.g. Note jammed in singulator))
    }

    // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
    /**
     * Indexer moves a game piece a given direction through the indexer subsystem given a direction, e.g. BACKWARD_INDEXER would mean pulling 
     * a held game piece from the indexing chamber back out to the intake.
     * @param Demand Directional Demand, which way for a game piece to be moved.
     */
    public synchronized void set(final Direction Demand) {
        if (getHoldingNote()) {
            switch(Demand) {
                case FORWARD_CANNON:
                //  MOTORS[] spin at Measurements.INDEXER_MOTOR_SPEED for Measurements.INDEXER_MOTOR_DURATION;
                //  Tell CANNON to continueWithFireSequence;
                    break;
                case BACKWARD_INDEXER:
                //  MOTORS[] spin at -(Measurements.INDEXER_MOTOR_SPEED) for Measurements.INDEXER_MOTOR_DURATION;
                //  Tell INTAKE to do intake sequence backwards (negative speed, same time, etc etc);
                    break;
                default:
                //  STOP ALL INDEXER MOTORS
                    break;
            }
        }
        else {
            switch(Demand) {
                case FORWARD_INDEXER:
                    //  MOTORS[] spin at Measurements.INDEXER_MOTOR_SPEED for Measurements.INDEXER_MOTOR_DURATION;
                    break;
                case FORWARD_CANNON:
                    //  Tell CANNON to abortFireSequence;
                    break;
                case BACKWARD_INDEXER:
                //  MOTORS[] spin at -(Measurements.INDEXER_MOTOR_SPEED) for Measurements.INDEXER_MOTOR_DURATION;
                //  Tell INTAKE to do intake sequence backwards (negative speed, same time, etc etc);
                    break;
                default:
                //  STOP ALL INDEXER MOTORS
                    break;
            }
        }
    }

    // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
    /**
     * Provides a boolean representation of if this indexer is currently holding a not or not.
     * @return Boolean of if a note is being held
     */
    public Boolean getHoldingNote() {
        return HasNote;
    }
    
    /**
     * Retrieves the existing instance of this static utility class
     * @return Utility class's instance
     */
    public static synchronized IndexerSubsystem getInstance() {
        if (java.util.Objects.isNull(Instance))
            Instance = new IndexerSubsystem();
        return Instance;
    }
}