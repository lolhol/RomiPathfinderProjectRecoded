// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemDrivetrain extends SubsystemBase {

    public final double kCountsPerRevolution = 1440.0;
    public final double kWheelDiameterInch = 2.75591; // 70 mm

    // The Romi has the left and right motors set to
    // PWM channels 0 and 1 respectively
    public final Spark m_leftMotor = new Spark(0);
    public final Spark m_rightMotor = new Spark(1);

    // The Romi has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    public final Encoder m_leftEncoder = new Encoder(4, 5);
    public final Encoder m_rightEncoder = new Encoder(6, 7);

    private double curSpeedPerc = 0;
    public boolean turning = false;
    public boolean isGoing = false;

    // Set up the differential drive controller
    private final DifferentialDrive m_diffDrive = new DifferentialDrive(
            m_leftMotor,
            m_rightMotor);

    /**
     * Creates a new RomiDrivetrain.
     */
    public SubsystemDrivetrain() {
        // Use inches as unit for encoder distances
        m_leftEncoder.setDistancePerPulse(
                (Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
        m_rightEncoder.setDistancePerPulse(
                (Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
        resetEncoders();

        // Invert right side since motor is flipped
        m_rightMotor.setInverted(true);
    }

    public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
        // System.out.println(xaxisSpeed + " | " + zaxisRotate);
        m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getLeftDistanceInch() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistanceInch() {
        return m_rightEncoder.getDistance();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_diffDrive.feed();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    // Speed -> -100% to 100%
    public void forwardBackward(double speed) {
        if (speed < -100 || speed > 100) {
            return;
        }

        double speedNeeded = speed / 100;
        this.arcadeDrive(-speedNeeded, 0);
        curSpeedPerc = speed;

        isGoing = speed != 0;
    }

    // Speed -> 1% - 100%
    public void turn(boolean isRight, double speed) {
        if (speed < 0 || speed > 100) {
            return;

        }

        double driveSpeed = speed / 100;
        if (isRight) {
            this.arcadeDrive(0, -driveSpeed);
        } else {
            this.arcadeDrive(0, driveSpeed);
        }

        this.curSpeedPerc = speed;
        this.turning = speed > 0;
    }

    public double curTurning() {
        if (turning) {
            return this.curSpeedPerc;
        }

        return 0;
    }

    public double curSpeed() {
        return this.curSpeedPerc;
    }
}
