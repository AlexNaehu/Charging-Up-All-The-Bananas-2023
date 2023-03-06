package frc.robot.Mechanisms;


//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.IMotorController;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot.BananaConstants;



public class BananaClaw {
    
    private static WPI_VictorSPX leftFinger;

    private int leftFinger_ID = 25;
    private int rightFinger_ID = 29;

    public static boolean intakeOpen = true;

    public BananaClaw()
    {

        leftFinger = new WPI_VictorSPX(leftFinger_ID);

    }

    public static void changeClawState()
    {
        intakeOpen = !intakeOpen;
    }
    public static void closeClaw(double power)
    {
        leftFinger.set(-power);
    }

    public static void openClaw(double power)
    {
        leftFinger.set(power);
    }

    public boolean isIntakeOpen()
    {
        return intakeOpen;
    }

    public double getLeftFingerPower()
    {
        return leftFinger.get();
    }
    


}

