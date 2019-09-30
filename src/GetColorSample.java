
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GetColorSample {
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
	SampleProvider light = colorSensor.getMode("Red");
	//SensorMode colorID= colorSensor.getColorIDMode(); 
	
	float[] sample = new float[light.sampleSize()];
	/*ArrayList<Float> blackVar = new ArrayList<Float>();
	ArrayList<Float> whiteVar = new ArrayList<Float>();
	*/
	static ColorSample cSample = new ColorSample();
	
	
	public void getColorVal(){
		while(!Button.LEFT.isDown()){
			if(Button.UP.isDown()){
				
				light.fetchSample(sample, 0);
				
				
				cSample.setBlackVal(sample[0]);
				
				LCD.drawString("black: "+GetColorSample.cSample.getBlackVal(), 0, 1);
				
				Delay.msDelay(1000);
				
			}
			else if(Button.DOWN.isDown()){
				light.fetchSample(sample, 0);
				
				cSample.setWhiteVal(sample[0]);
				LCD.drawString("white: "+GetColorSample.cSample.getWhiteVal(), 0, 2);
				
				Delay.msDelay(1000);
			}
			
		}
		colorSensor.close();
		
	}
	
	
}
