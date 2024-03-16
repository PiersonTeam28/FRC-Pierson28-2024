package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX motor;
    private MotionMagicVoltage motionMagicController = new MotionMagicVoltage(0);
    

    public Arm() {
        // TO DO: Perfect slot configs for motor control
        motor = new TalonFX(23);
        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10;
        motionMagicConfigs.MotionMagicAcceleration = 5;
        motionMagicConfigs.MotionMagicJerk = 200;

        motor.getConfigurator().apply(talonFXConfigs);
        motor.setPosition(0);
    }

    public void moveToPose(double pose){
        System.out.println(motor.setControl(motionMagicController.withPosition(pose)));
    }

    private Command move(BooleanSupplier press, double speed){
        return new Command(){
            public void initialize() {
                motor.set(speed);
            }
        
            public void end(boolean interrupted) {
                motor.stopMotor();
            }

            public boolean isFinished() {
                if (motor.getPosition().getValueAsDouble() <= -85){
                    motor.setPosition(-85);
                    return true;
                }
                if (motor.getPosition().getValueAsDouble() > 0){
                    motor.setPosition(0);
                    return true;
                }
                if (!press.getAsBoolean())
                    return true;
                return false;
            }
        };
    }

    public Command up(BooleanSupplier press){
        return move(press, -.4);
    }

    public Command down(BooleanSupplier press){
        return move(press, .4);
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", motor.getPosition().getValueAsDouble());
    }
}