import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.robotics.navigation.DifferentialPilot;


public class PIDController {
	EV3ColorSensor colorSensor2 = new EV3ColorSensor(SensorPort.S3);
	EV3UltrasonicSensor ul = new EV3UltrasonicSensor(SensorPort.S4);
	DifferentialPilot robot = new DifferentialPilot(5.6f,11.2f , Motor.A, Motor.C);
	//getDistanceMode the distance (in metres) to an object in front of the sensor
	SampleProvider bump = ul.getDistanceMode();
	float[] sampleDis = new float[1];
	SampleProvider light2 = colorSensor2.getMode("Red");
	SensorMode colorID= colorSensor2.getColorIDMode(); 
	
	float[] sampleColorID = new float[colorID.sampleSize()];
	
	float blackVal = GetColorSample.cSample.getBlackVal()*100;
	
	
	float whiteVal = GetColorSample.cSample.getWhiteVal()*100;
	float[] sample2 = new float[light2.sampleSize()];
	float bwVal = (blackVal+whiteVal)/2;
	//float kP = 100/(whiteVal-blackVal);
	//float kP = 0.6f;
	float kP = 0.6f;
	//float kI = 0.00005f;
	float kI = 0.00005f;
	//float kD = 0.05f;
	float kD = 0.05f;
	int defaultPower = 50;
	float powerA;
	float powerC;
	int maxPower = 100;
	int minPower = -100;
	float error;
	float pTurn;
	float iTurn;
	float integral=0;
	float lastError = 0;
	
	float derivative = 0;
	float dTurn;
	boolean obstacle=false;
	int countB=0;
	int countW=0;
	boolean turn45=false;
	
	/*void test(){
		while(!Button.ENTER.isDown() && !obstacle){
			light2.fetchSample(sample2,0);
			Motor.A.forward();
			Motor.C.forward();
			if(sample2[0]<=bwVal){
				countB++;
			}
			else if(sample2[0]>bwVal){
				countW++;
			}
			
		}
		System.out.println("white="+countW);
	}*/
	void run(){
		//!Button.ENTER.isDown()
		while(!Button.ENTER.isDown()){
			System.out.println("initial:  "+turn45+"\n");
				followLine();
				
				detectObs();
				
				
				
				detectRed();				
			
			
			
			
		}
		colorSensor2.close();
		ul.close();
	}
	
	
	void followLine(){
		light2.fetchSample(sample2,0);
		if(sample2[0]*100<blackVal)
			sample2[0] = blackVal/100;
		if(sample2[0]*100>whiteVal)
			sample2[0] = whiteVal/100;
		//motor A > motor C=>turn right
		error = sample2[0]*100 -bwVal;
		//Proportion 
		pTurn = kP*error;
		
		//Integral
		integral = integral + error;
		iTurn = kI*integral;
		//Derivative
		derivative = error - lastError;
		dTurn = kD*derivative;
		powerA = -pTurn+iTurn+dTurn+defaultPower;
		powerC = pTurn+iTurn+dTurn+defaultPower;
		
		if(powerA<minPower){
			powerA = -100;
		}
		else if(powerC>maxPower){
			powerC = 100;
		}
		//System.out.println("power A: "+ powerA);
		//System.out.println("power C: "+ powerC);
		
		Motor.A.setSpeed(5*powerA);
		Motor.C.setSpeed(5*powerC);
		
		Motor.A.forward();
		Motor.C.forward();
		lastError = error;
		
	}
	void detectRed(){
		
		colorID.fetchSample(sampleColorID, 0);
		if(sampleColorID[0]==0){
			Motor.A.stop();
			Motor.C.stop();
			//if(!turn45){
				robot.steer(200f,35f  );
				turn45 =true;
				
				Delay.msDelay(1000);
			//}
			//else{
				
				
			//}
			
			
		}
		
		
	}

	void detectObs(){
		bump.fetchSample(sampleDis, 0);
		if(sampleDis[0]<0.15){
			
			Motor.A.stop();
			Motor.C.stop();
			
				
					robot.steer(200f,155f);
					turn45 =true;
					Delay.msDelay(1000);
				 
				
			
			//detect if there's a black line
			
			
//			powerA =0;
//			powerC=0;
//			error=0;
//			integral=0;
//			derivative=0;
			//Motor.A.rotate(45,true);
		}
		
		
				
	}
		
		
		
	
}
