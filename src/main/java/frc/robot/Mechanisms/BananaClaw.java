package frc.robot.Mechanisms;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.*;
import edu.wpi.first.wpilibj.AnalogInput;


public class BananaClaw {
    
    private static WPI_VictorSPX leftFinger;
    private static WPI_VictorSPX rightFinger;

    private int leftFinger_ID = 24;
    private int rightFinger_ID = 25;

    public static boolean intakeOpen = true;

    public BananaClaw()
    {

        leftFinger = new WPI_VictorSPX(leftFinger_ID);
        rightFinger = new WPI_VictorSPX(rightFinger_ID);

    }

    public static void changeClawState()
    {
        intakeOpen = !intakeOpen;
    }
    public static void closeClaw(double power)
    {
        leftFinger.set(power);
        rightFinger.set(-power);
    }

    public static void openClaw(double power)
    {
        leftFinger.set(-power);
        rightFinger.set(power);
    }

    public boolean isIntakeOpen()
    {
        return intakeOpen;
    }

    public double getLeftFingerPower()
    {
        return leftFinger.get();
    }

    public double getRightFingerPower()
    {
        return rightFinger.get();
    }
    


}

