package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;

public class ElevatorConstants {
    protected static final Motor
            ELEVATOR_LEFT_MOTOR = MotorFactory.createSpark("Elevator Left Motor", 0, MotorProperties.SparkType.MAX),
            ELEVATOR_RIGHT_MOTOR = MotorFactory.createSpark("Elevator Right Motor", 1, MotorProperties.SparkType.MAX);

    protected static final Sensor
            TOP_BEAM_BREAKER = SensorFactory.createDigitalInput("Top Beam Breaker", 2),
            BUTTON_BEAM_BREAKER = SensorFactory.createDigitalInput("Button Beam Breaker", 3);

    protected static final double TOP_ELEVATOR_METERS_POSITION = 1;

    private static final MotorConfiguration ELEVATOR_LEFT_MOTOR_CONFIGURATION = new MotorConfiguration();

    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationSlot = new MotorProperties.Slot(70, 0, 0, 0, 0, 1.3129999999999662);

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxAcceleration = 15;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxVelocity = 10;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxJerk = 150;

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ELEVATOR,
                DCMotor.getNeoVortex(2),
                60,
                24,
                0.2,
                0.1,
                1.1,
        false
        );

// issue: The Elevator stops at a specific position for unknown
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);

        ELEVATOR_LEFT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
        ELEVATOR_RIGHT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
    }
}