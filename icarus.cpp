/*
Code initial écrit par Colin Rousseau, dans le cadre du projet Icarus, grandement inspiré de la logique de contrôle amenée par Carbon Aeronautics
*/

#include <Wire.h>
#include <PulsePosition.h>
#include <Fusion.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>


FusionAhrs ahrs; // initialize fusion algo
// Set AHRS algorithm settings

const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = 0.3f,
    .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 10.0f,
    .magneticRejection = 10.0f,
    .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
};

float RatePitch, RateRoll, RateYaw;
float madwick_roll, madwick_pitch;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

PulsePositionInput ReceiverInput(RISING); //No idea as of now
float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0}; //Necessary?

struct receiver_values {
    float pitch = 0;
    float roll = 0;
    float yaw_rate = 0;
    float throttle;
}
int ChannelNumber=0; //Huh... I'd prefer use name than numbers for received values

//Battery management variables
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed=0;
float BatteryDefault=1300;



// PID Loop variables
int MaxPIDvalues = 400
    //angle pid
float DesiredRoll, DesiredPitch;
float ErrorRoll, ErrorPitch;
float PrevErrorRoll, PrevErrorPitch;
float PrevItermRoll, PrevItermPitch;
    //rate pid
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
    
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PIDReturn[]={0, 0, 0};

//PID parameters
    //rate
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
    //angle
float PRoll= 2. ; float PPitch=PRoll;
float IRoll= 0. ; float IPitch=IRoll;
float DRoll= 0. ; float DPitch=DRoll;

//motors inputs
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

//timer
uint32_t LoopTimer;

void read_mpu(){
    
}

void run_initial_gyro_calibration(){
    //take 2000 measurements before settling errors
    int calibration_measurements = 2000;
    for (RateCalibrationNumber=0; RateCalibrationNumber<calibration_measurements; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
    }
    RateCalibrationRoll/= calibration_measurements;
    RateCalibrationPitch/= calibration_measurements;
    RateCalibrationYaw/= calibration_measurements;
}

void read_receiver(){

}

void initialize_communication(){

}

//void compute_desired_rates(){} designed for rate control

//void compute_rate_errors(){} designed for rate control

void compute_desired_angles(){
    //assuming receiver values between 1000 and 2000 (pmw signal)
    //assuming max_tile of 25 degrees (500*0.05)
    DesiredRoll = 0.05*(receiver_values.roll-1500);
    DesiredPitch = 0.05*(receiver_values.pitch-1500);
    DesiredRateYaw = 0.15*(receiver_values.yaw_rate-1500); // limite rate control to 75 deg/s

}


void pid_equation(float Error, float P, float I, float PrevError, float PrevIterm){
    uint32_t time_passed = (micros()- LoopTimer())/1000000;

    //proportionnal
    float Pterm = P*Error;

    //integral
    float Iterm = PrevIterm + I*(Error+PrevError)*time_passed/2; // define time_passed
    //limit drifting I term
    if (Iterm > MaxPIDvalues) Iterm = MaxPIDvalues;
    else if (Iterm < -MaxPIDvalues) Iterm = -MaxPIDvalues;

    //derivative
    float Dterm = D*(Error-prevError)/time_passed;

    //sum PID values
    float PIDoutput = Pterm + Iterm + Dterm;

    //Limit PID output to be maxPIDvalues
    if (PIDOutput > MaxPIDvalues) PIDOutput= MaxPIDvalues;
    else if (PIDOutput < -MaxPIDvalues) PIDOutput= -MaxPIDvalues;

    //assign_return
    PIDReturn[0]=PIDOutput;
    PIDReturn[1]=Error;
    PIDReturn[2]=Iterm;
}

void apply_PID_loops(){
    //compute angles errror
    ErrorRoll = DesiredRoll- madwick_roll;
    ErrorPitch = DesiredPitch - madwick_pitch;

    //Use PID to compute roll rate and pitch rate (outer control loop)
    pid_equation(ErrorRoll, PRoll, IRoll, DRoll, PrevErrorRoll, PrevItermRoll);
        DesiredRateRoll = PIDReturn[0];
        PrevErrorRoll = PIDReturn[1];
        PrevItermRoll = PIDReturn[2];

    pid_equation(ErrorRoll, PRoll, IRoll, DRoll, PrevErrorRoll, PrevItermRoll);
        DesiredRatePitch = PIDReturn[0];
        PrevErrorPitch = PIDReturn[1];
        PrevItermPitch = PIDReturn[2];

    //Evalutate rates errors
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    //use PID to compute roll, pitch, yaw motor input
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];

    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch=PIDReturn[0]; 
        PrevErrorRatePitch=PIDReturn[1]; 
        PrevItermRatePitch=PIDReturn[2];

    pid_equation(ErrorRateYaw, PRateYaw,
        IRateYaw, DRateYaw, PrevErrorRateYaw,
        PrevItermRateYaw);
        InputYaw=PIDReturn[0]; 
        PrevErrorRateYaw=PIDReturn[1]; 
        PrevItermRateYaw=PIDReturn[2];
}

void limit_throttle(){
    if (InputThrottle > 1800) InputThrottle = 1800;
}

void reset_pid(){
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    
    PrevErrorRoll=0; PrevErrorPitch=0;
    PrevItermRoll=0; PrevItermPitch=0; 
}

void battery_voltage(){

}

void handle_battery_voltage_logic(){

}

void apply_calibration(){

    RateRoll-= RateCalibrationRoll;
    RatePitch-= RateCalibrationPitch;
    RateYaw-= RateCalibrationYaw;

}

void compute_motor_inputs(){
    //1. control mixer
    float multiplication_factor = 1.024; // check carbon aeronautics for more info
    MotorInput1 = multiplication_factor*(InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = multiplication_factor*(InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = multiplication_factor*(InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = multiplication_factor*(InputThrottle + InputRoll - InputPitch + InputYaw);

    //2. Limit max and min thrust
    if (MotorInput1 > 2000)MotorInput1 = 1999;
    if (MotorInput2 > 2000)MotorInput2 = 1999; 
    if (MotorInput3 > 2000)MotorInput3 = 1999; 
    if (MotorInput4 > 2000)MotorInput4 = 1999;

    int ThrottleIdle=1180; // minimum spinning throttle //move up?
    if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;

    
    //3. Impose motor cutoff below threshold
        // rest PID values
    int ThrottleCutOff=1000; //cutoff throttle under 5%
    if (ReceiverValue[2]<1050) {
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        reset_pid();
    }
    
    // Sent commands computed
    analogWrite(1,MotorInput1);
    analogWrite(2,MotorInput2);
    analogWrite(3,MotorInput3); 
    analogWrite(4,MotorInput4);
}

void apply_fusion(){
    const FusionVector gyroscope = {gyroX, GyroY, GyroZ}; // replace this with actual gyroscope data in degrees/s
    const FusionVector accelerometer = {AccX, AccY, AccZ}; // replace this with actual accelerometer data in g

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);//replace sample_period by time passed

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

}

void setup(){
    //normal initialization
    //Setup inputs and ouputs, pins etc

    Wire.setClock(400000); //set micro- controller fast mode
    Wire.begin();
    delay(250);

    FusionAhrsInitialise(&ahrs);

    run_initial_gyro_calibration()


    /* Set operating frequency? 250 Hz
    analogWriteFrequency(1, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12);
    */

   battery_voltage();
   //Battery voltage start operating (intial voltage, etc)
   //
   //
   //

   //initialize comm
   ReceiverInput.begin(14);
   //Prevent start of mainloop until after low throttle is positionned
   while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    read_receiver();
    delay(4);
   }

   //Initialize timer
   LoopTimer=micros();
}

void loop(){

    read_mpu();
    apply_calibration();
    apply_fusion();

    read_receiver();

    compute_desired_angles();

    pid_equation();//roll
    pid_equation();//pitch

    pid_equation();//roll_rate
    pid_equation();//pitch_rate
    pid_equation();//yaw_rate

    //enforce 250 hz frequency, juste after pid_cause thats_where time_passed is used
    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();

    limit_throttle();

    compute_motor_inputs();

    battery_voltage();

    compute_remaining_battery();
    battery_low_handler();

}