package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    public Command down(){
        return new Command(){
                        // Called when the command is initially scheduled.
            @Override
            public void initialize() {
                
            }

            // Called every time the scheduler runs while the command is scheduled.
            @Override
            public void execute() {
                
            }

            // Called once the command ends or is interrupted.
            @Override
            public void end(boolean interrupted) {

            }

            // Returns true when the command should end.
            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public void up(){
        motor.setVoltage(-2);
    }
    
    @Override
    public void periodic(){
        
    }
}