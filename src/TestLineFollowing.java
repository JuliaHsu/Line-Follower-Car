import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.port.SensorPort;


public class TestLineFollowing {
	public static void main(String[] args){
		
		GetColorSample colorValRes = new GetColorSample();
		colorValRes.getColorVal();
		
		
		
		Delay.msDelay(2000);
		PIDController controller = new PIDController();
		
		controller.run();
		
	}

}
