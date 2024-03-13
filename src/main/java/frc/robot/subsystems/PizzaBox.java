package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.SpiReadAutoReceiveBufferCallback;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootCommand;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class PizzaBox extends SubsystemBase {
    private CANSparkMax leftShooterMotor;
    private CANSparkMax rightShooterMotor;
    private TalonFX holdingMotor;

    public PizzaBox() {
        leftShooterMotor = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless);
        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.follow(leftShooterMotor, true);
        holdingMotor = new TalonFX(19);

    }

    public void startShootMotors(){
        leftShooterMotor.set(-1);
    }

    public void startIntakeMotors(){
        System.out.println("intaking");
        leftShooterMotor.set(1);
        holdingMotor.setVoltage(5);
    }

    public void startDispenseMotors(){
        leftShooterMotor.set(-.5);
        holdingMotor.setVoltage(-5);
    }

    public void releaseHoldingMotor(){
        holdingMotor.set(-1);
    }

    public void stopAllMotors(){
        System.out.println("stop");
        leftShooterMotor.stopMotor();
        holdingMotor.stopMotor();
    }

    public ShootCommand shoot(){
        return new ShootCommand(this);
    }

    public InstantCommand intake(){
        return new InstantCommand(() -> this.startIntakeMotors(), this);
    }

    public InstantCommand stop(){
        return new InstantCommand(() -> this.stopAllMotors(), this);
    }

    public InstantCommand dispense(){
        return new InstantCommand(() -> this.startDispenseMotors());
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("left shooter motor out", leftShooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("right shooter motor out", rightShooterMotor.getAppliedOutput());
    }
}