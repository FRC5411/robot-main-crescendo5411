// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;
// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogTrigger;

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
    private static final CANSparkMax INTAKE_INDEXER;  
    private static final CANSparkMax CANNON_INDEXER;
    private static final AnalogTrigger INTAKE_RECEIVER;
    private static final AnalogTrigger CANNON_RECEIVER;
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

        INTAKE_RECEIVER = new AnalogTrigger(Constants.Ports.INTAKE_INDEXER_RECEIVER_ID);
        CANNON_RECEIVER = new AnalogTrigger(Constants.Ports.CANNON_INDEXER_RECEIVER_ID);

        INTAKE_INDEXER = new CANSparkMax(Constants.Ports.INTAKE_INDEXER_MOTOR_ID, MotorType.kBrushless);
        CANNON_INDEXER = new CANSparkMax(Constants.Ports.CANNON_INDEXER_MOTOR_ID, MotorType.kBrushless);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
      Constants.Objects.ODOMETRY_LOCKER.lock();
      HasNote = !(INTAKE_RECEIVER.getInWindow()|| CANNON_RECEIVER.getInWindow());
      Constants.Objects.ODOMETRY_LOCKER.lock();
    }
    
    /**
     * Closes this instance and all held resources immediately 
     */
    public synchronized void close() {
      INTAKE_INDEXER.close();
      CANNON_INDEXER.close();
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
     * Indexer moves a game pice a given direction through the indexer subsystem given a direction, e.g. Backward Indexer would mean pulling a 
     * held game piece from the indexing chamber back out to the indexer.
     * @param Demand Directional Demand, which way for a game piece to be moved.
     */
    public synchronized void set(final Direction Demand) {
      switch(Demand) {
        case BACKWARD_INDEXER:
          break;
        case BACKWARD_CANNON:
          break;
        case FORWARD_INDEXER:
          break;
        case FORWARD_CANNON:
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