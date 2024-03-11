// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.elevator;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ElevatorModuleStatusContainer {
    public double Speed_Decimal = 0;
    public double Velocity_MpSec = 0;
    public double Position_RD = 0;
    public double Volts = 0;
}
