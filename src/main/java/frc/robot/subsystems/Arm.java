package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX motor;
    

    public Arm() {
        motor = new TalonFX(-4);
        
    }

    public void doSomething(){
        motor.setControl(new PositionVoltage(0, 0, false, 0, 0, false, false, false));
    }

    public void stopSomething(){

    }
    
    
    @Override
    public void periodic(){
        
    }
}