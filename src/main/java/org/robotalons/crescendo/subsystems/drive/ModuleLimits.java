package org.robotalons.crescendo.subsystems.drive;

public record ModuleLimits(
    double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}
