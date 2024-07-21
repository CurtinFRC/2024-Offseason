package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
 

public class LED {
  private static final PWM LEDPWM = new PWM(0);

  public LED () {
  
  }

  public static void Intake() {
    LEDPWM.setSpeed(0.59); // dark red //
  }

  public static void Outake() {
    LEDPWM.setSpeed(0.63); // red orange, 0.65 for normal orange //
  }

  public static void Spinup(){
    LEDPWM.setSpeed(0.69); // yellow //
  }

  public static void Stop(){
    LEDPWM.setSpeed(0.73); // lime //
  }

  public static void Maintain(){
    LEDPWM.setSpeed(0.57); // hot pink //
  }

}
