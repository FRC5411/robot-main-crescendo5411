// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.lib.climb;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[ClimbStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the climb information of a given robot.
 */
@AutoLog
public class ClimbStatusContainer {
    public double posistion = (0d);
    public double velocity = (0d);
    public double temperature = (0d);

}
