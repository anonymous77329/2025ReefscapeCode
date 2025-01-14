package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.StaticFrictionCharacterization;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends GenericSubsystem {
    private final ElevatorMechanism2d mechanism = new ElevatorMechanism2d("Elevator Mechanism", 1);
    private final StaticFrictionCharacterization frictionCheck = new StaticFrictionCharacterization(this, ELEVATOR_LEFT_MOTOR, 0);

    public Command setTargetPosition(double targetPositionMeters){
        return Commands.runEnd(() -> {
            setMotorPosition(metersToRotations(targetPositionMeters));
            printPose(targetPositionMeters);  // prints the elevator 3DPose
        }, this::stopMotors, this);
    }

    public void periodic() {
//        if (TOP_BEAM_BREAKER.get() == 0) {
//            ELEVATOR_LEFT_MOTOR.setMotorEncoderPosition(0);
//            ELEVATOR_RIGHT_MOTOR.setMotorEncoderPosition(0);
//        }
//
//        if (TOP_BEAM_BREAKER.get() == TOP_ELEVATOR_METERS_POSITION) {
//            ELEVATOR_LEFT_MOTOR.setMotorEncoderPosition(TOP_ELEVATOR_METERS_POSITION);
//            ELEVATOR_RIGHT_MOTOR.setMotorEncoderPosition(TOP_ELEVATOR_METERS_POSITION);
//        }
    }

    public Command giveMeThefrictionCheckYoooo() {
        return frictionCheck;
    }

    private void printPose(double targetPositionMetes) {
        Pose3d current3DPose = new Pose3d(0, 0, targetPositionMetes, new Rotation3d(0, 0, 0));
        Logger.recordOutput("Elevator", current3DPose);
        mechanism.updateCurrentPosition(ELEVATOR_LEFT_MOTOR.getSystemPosition());
        mechanism.updateTargetPosition(targetPositionMetes);
        Logger.recordOutput("Elevator works??", ELEVATOR_LEFT_MOTOR.getSystemPosition());
    }

    private void setMotorPosition(double targetPosition) {
        ELEVATOR_LEFT_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
        ELEVATOR_RIGHT_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }

    private void stopMotors() {
        ELEVATOR_LEFT_MOTOR.stopMotor();
        ELEVATOR_RIGHT_MOTOR.stopMotor();
    }

    private double metersToRotations(double meters) {
        double pulleyDiameter = 0.05;
        double pulleyCircumference = Math.PI * pulleyDiameter;

        return meters / pulleyCircumference;
    }
}
