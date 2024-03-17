package frc.robot.wrappers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MonitoredPIDController extends PIDController{
    private String name;
    private double lastOutput;
    private boolean disabled;

    public MonitoredPIDController(double p, double i, double d, String name){
        super(p, i, d);
        this.name = name;
        lastOutput = 0;
        disabled = false;
    }

    @Override
    public double calculate(double measurement){
        if (!disabled){
            SmartDashboard.putNumber(name + " Input", measurement);
            double output = super.calculate(measurement);
            SmartDashboard.putNumber(name + " Output", output);
            return output;
        }
        return 0;
    }
    
    @Override
    public double calculate(double measurement, double setpoint){
        if (!disabled){
            SmartDashboard.putNumber(name + " Input", measurement);
            SmartDashboard.putNumber(name + " Setpoint", setpoint);
            lastOutput = super.calculate(measurement, setpoint);
            SmartDashboard.putNumber(name + " Output", lastOutput);
            return lastOutput;
        }
        return 0;
    }

    @Override 
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("MonitoredPIDController");
        builder.setSafeState(()-> {});
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("Setpoint", this::getSetpoint, null);
        builder.addDoubleProperty("Output", () -> lastOutput, null);
        System.out.println("################## initSendable done");
    }

    public void disable(){
        disabled = true;
    }

    public void enable(){
        disabled = false;
    }

    public boolean isDisabled(){
        return disabled;
    }
}