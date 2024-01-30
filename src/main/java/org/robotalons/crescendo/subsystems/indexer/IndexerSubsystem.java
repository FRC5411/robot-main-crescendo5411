// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.io.Closeable;

// ------------------------------------------------------------[Indexer Subsystem]---------------------------------------------------------- //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes from and to the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */

public class IndexerSubsystem extends SubsystemBase implements Closeable {
    // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
    private static final CANSparkMax INDEXER_INTAKE_MOTOR;  
    private static final CANSparkMax INDEXER_CANNON_MOTOR;
    private static final DigitalInput INDEXER_INTAKE_RECEIVER;
    private static final DigitalInput INDEXER_CANNON_RECEIVER;

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

        INDEXER_INTAKE_RECEIVER = new DigitalInput(Constants.Ports.INDEXER_INTAKE_RECEIVER_ID);
        INDEXER_CANNON_RECEIVER = new DigitalInput(Constants.Ports.INDEXER_CANNON_RECEIVER_ID);

        INDEXER_INTAKE_MOTOR = new CANSparkMax(Constants.Ports.INDEXER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        INDEXER_CANNON_MOTOR = new CANSparkMax(Constants.Ports.INDEXER_CANNON_MOTOR_ID, MotorType.kBrushless);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
      Constants.Objects.ODOMETRY_LOCKER.lock();
      HasNote = !(INDEXER_INTAKE_RECEIVER.get()|| INDEXER_CANNON_RECEIVER.get());
      Constants.Objects.ODOMETRY_LOCKER.lock();
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
      FORWARD_CANNON,
      FORWARD_INDEXER,
      BACKWARD_CANNON,
      BACKWARD_INDEXER,
    }

    // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
    /**
     * Indexer moves a game piece a given direction through the indexer subsystem given a direction, e.g. Backward Indexer would mean pulling 
     * a held game piece from the indexing chamber back out to the intake.
     * @param Demand Directional Demand, which way for a game piece to be moved.
     */
    public synchronized void set(final Direction Demand) {
        switch(Demand) {
            case FORWARD_INDEXER:   /* will be called by the intake subsystem after completing a pickup.
                if (!(getHoldingNote())) {
                    INDEXER_INTAKE_MOTOR spin at INDEXER_INTAKE_MOTOR_SPEED for INDEXER_INTAKE_MOTOR_DURATION;
                } */
                break;
            case FORWARD_CANNON:    /* will be called by the cannon subsystem in order to put a Note in firing position.
                if (getHoldingNote()) {
                    INDEXER_CANNON_MOTOR spin at INDEXER_CANNON_MOTOR_SPEED for INDEXER_CANNON_MOTOR_DURATION;
                    Tell CANNON to continueWithFireSequence;
                }
                else {
                    Tell CANNON to abortFireSequence;
                } */
                break;
            case BACKWARD_INDEXER:  /* will be called by the operator/copilot if a Note needs to be put back on the ground (e.g. Note jammed in singulator).
                if (getHoldingNote())
                {
                    INDEXER_INTAKE_MOTOR spins at -INDEXER_INTAKE_MOTOR_SPEED for INDEXER_INTAKE_MOTOR_DURATION;
                    Tell INTAKE to do intakeSequence but backwards (negative speed, same time, etc etc);
                } */
                break;
            default:
                break;
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