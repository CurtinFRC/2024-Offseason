package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants;
 

public final class LED {
  private static final PWM ledpwm = new PWM(Constants.LEDport);

  private LED () {
  }

  public static void Intake() {
    ledpwm.setSpeed(0.59); // dark red //
  }

  public static void Outake() {
    ledpwm.setSpeed(0.63); // red orange, 0.65 for normal orange //
  }

  public static void Spinup(){
    ledpwm.setSpeed(0.69); // yellow //
  }

  public static void Stop(){
    ledpwm.setSpeed(0.73); // lime //
  }

  public static void Maintain(){
    ledpwm.setSpeed(0.57); // hot pink //
  }

}
