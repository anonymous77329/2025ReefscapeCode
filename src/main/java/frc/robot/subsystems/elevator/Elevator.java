package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.StaticFrictionCharacterization;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends GenericSubsystem {
    public Command setTargetPosition(double targetPositionMeters){
        return Commands.runEnd(() -> {
            setMotorPosition(Conversions.metresToRotations(targetPositionMeters, WHEEL_DIAMETER));
            printPose(targetPositionMeters);
        }, this::stopMotors, this);
    }

    public void periodic() {
        if (TOP_BEAM_BREAK.get() == 0) {
            ELEVATOR_LEFT_MOTOR.setMotorEncoderPosition(0);
            ELEVATOR_RIGHT_MOTOR.setMotorEncoderPosition(0);
        }

        if (TOP_BEAM_BREAK.get() == ELEVATOR_MAX_EXTENSION_METERS) {
            ELEVATOR_LEFT_MOTOR.setMotorEncoderPosition(ELEVATOR_MAX_EXTENSION_METERS);
            ELEVATOR_RIGHT_MOTOR.setMotorEncoderPosition(ELEVATOR_MAX_EXTENSION_METERS);
        }
    }

    private void printPose(double targetPositionMetes) {
        final Pose3d current3DPose = new Pose3d(0, 0, targetPositionMetes, new Rotation3d(0, 0, 0));
        Logger.recordOutput("Elevator", current3DPose);

        MECHANISM.updateCurrentPosition(ELEVATOR_LEFT_MOTOR.getSystemPosition());
        MECHANISM.updateTargetPosition(targetPositionMetes);
    }

    private void setMotorPosition(double targetPosition) {
        ELEVATOR_LEFT_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
        ELEVATOR_RIGHT_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetPosition);
    }

    private void stopMotors() {
        ELEVATOR_LEFT_MOTOR.stopMotor();
        ELEVATOR_RIGHT_MOTOR.stopMotor();
    }
}
