package frc.robot.Mechanisms;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;




public class BananaBrake {



    private static WPI_VictorSPX leftBrake;
    private static WPI_VictorSPX rightBrake;

    private int leftBrake_ID = 26;
    private int rightBrake_ID = 27;

    public static boolean brakeOn = false;

    public BananaBrake()
    {

        leftBrake = new WPI_VictorSPX(leftBrake_ID);
        rightBrake = new WPI_VictorSPX(rightBrake_ID);

    }

    public static void changeBrakeState()
    {
        brakeOn = !brakeOn;
    }
    public static void Brake(double power) //can only turn brake on, not off
    {
        leftBrake.set(power);
        rightBrake.set(-power);
    }


    public static boolean isBrakeOn()
    {
        return brakeOn;
    }

    public static double getLeftBrakePower()
    {
        return leftBrake.get();
    }

    public static double getRightBrakePower()
    {
        return rightBrake.get();
    }
}
