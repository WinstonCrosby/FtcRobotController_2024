package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;

@TeleOp(name = "TeleOp")
public class teleop_global extends OpMode {

    public IMU imu;

    //name motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor ArmMotor;
    DcMotor ExtendArm;

    CRServo grabber;
    Servo wrist;

    TouchSensor ResetExtend;
    float PrevButton;

    double desVal = 0;
    double wanVal = 0;

    double prevExt = 0;

    //Define set arm positions
    float armLow = 0;
    float armGrab = 800;
    float armMed = 1750;
    float armHigh = 2100;
    float armHang = 3000;

    int Height = 0;

    //Define variables for current arm position and desired arm position
    float currentArmPosition;
    float desArmPosition = 0;
    float prevArmError = 0;

    //Define Wheel Speed Vals
    float x = 0;
    float y = 0;
    float t = 0;

    //Define Control Mode
    int ControlMode = 0;

    //Define flag for incrementing arm mode
    boolean armChange = true;

    //Define yaw variables
    double initial_Yaw;
    double real_Yaw;

    //initialize
    @Override
    public void init() {
        //Define and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT))
        );

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");

        grabber = hardwareMap.get(CRServo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");
        ResetExtend = hardwareMap.get(TouchSensor.class, "ResetExtend");

        telemetry.addData("Hardware", "much gud");
    }

    @Override
    public void start() { wrist.setPosition(0.39); }


    //loop start
    @Override
    public void loop () {


        //-------------------------Input Block--------------------------------//
        //-------------------Read all inputs in here--------------------------//
        //--------------------------------------------------------------------//

        if (gamepad2.y && armChange && Height < 4){
            Height += 1;
            armChange = false;
        }
        else if(gamepad2.a && armChange && Height > 0){
            Height -= 1;
            armChange = false;
        }
        else if(!gamepad2.a && !gamepad2.y)
        {
            armChange = true;
        }

        //Check height
        telemetry.addData("Arm Height Mode", Height);

        if (ArmMotor.getCurrentPosition() > 1500){
            x = gamepad1.left_stick_x/4;
            y = gamepad1.left_stick_y/4;
            t = gamepad1.right_stick_x/4;
        }
        else {
            x = gamepad1.left_stick_x / 2;
            y = gamepad1.left_stick_y / 2;
            t = gamepad1.right_stick_x / 2;
        }

        float ea = -gamepad2.right_stick_y/2;

        //Check current arm position
        currentArmPosition = ArmMotor.getCurrentPosition();


        if (ResetExtend.isPressed()){
            ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            PrevButton = 1;
        }
        else {
            ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PrevButton = 0;
        }
        telemetry.addData("Sensor", ResetExtend.isPressed());

        double Yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        float Yaw_mod = (float)Yaw;

        telemetry.addData("IMU Angle", Yaw);
        telemetry.addData("Modified Angle", Yaw_mod);

        //--------------------------------------------------------------------//
        //-------------------------End Input Block----------------------------//
        //--------------------------------------------------------------------//

        //Modify x and y inputs to be globally referenced
        float x_mod = -y * (float)Math.sin(Yaw_mod) + x * (float)Math.cos(Yaw_mod);
        float y_mod = y * (float)Math.cos(Yaw_mod) + x * (float)Math.sin(Yaw_mod);

        //Function used to move the robot
        Drive(x_mod, y_mod, t);

        // Switch Arm Height (maybe)
        switch (Height)
        {
            case 0:
            {
                desArmPosition = armLow;
                telemetry.addData("Arm Value", "Arm is Down");
                break;
            }
            case 1:
            {
                desArmPosition = armGrab;
                telemetry.addData("Arm Value", "Arm is Grabbing");
                break;
            }
            case 2:
            {
                desArmPosition = armMed;
                telemetry.addData("Arm Value", "Arm is at Low Basket");
                break;
            }
            case 3:
            {
                desArmPosition = armHigh;
                telemetry.addData("Arm Value", "Arm is at High Basket");
                break;
            }
            case 4:
            {
                desArmPosition = armHang;
                telemetry.addData("Arm Value", "Arm is in Hang Position");
                break;
            }
        }
        //Function used to control the angle of the arm
        //Returns the previous error so that the variable is not overwritten every iteration
        prevArmError = Arm(desArmPosition, currentArmPosition, prevArmError);

        //Function used to extend or retract the arm
        Extend();

        //NewExtend();
        Grabber();

        //If B button held, make grabber spit out
        if (gamepad2.dpad_right)
        {
            wrist.setPosition(0.39);
        }


        if (gamepad2.dpad_left)
        {
            wrist.setPosition(0.74);
        }
    }

    public void Drive(float x_power, float y_power, float turn_power)
    {
        //Set motor power for each wheel motor
        motorFL.setPower(y_power - x_power - turn_power);
        motorFR.setPower(-y_power - x_power - turn_power);
        motorBL.setPower(y_power + x_power - turn_power);
        motorBR.setPower(-y_power + x_power - turn_power);
    }

    public float Arm(float desPosition, float currentPosition, float prevError)
    {

        //Move to init()?
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double kp = 0.0005;
        double kd = 0.0005;

        //Calculate error values for PID loop here
        float error = desPosition - currentPosition;
        float error_d = error - prevError;

        //Calculate motor power
        double motorPower = (kp * error) + (kd * error_d);

        //Set motor power
            ArmMotor.setPower(motorPower);

        //Print motor power
        telemetry.addData("Arm Value", ArmMotor.getCurrentPosition());


        return error;

        /*
        if (ArmMotor.getCurrentPosition() >= -10){
            if (ArmMotor.getCurrentPosition() < 3000) {
                ArmMotor.setPower(ea);
            }
        }
        double mvVal = ArmMotor.getCurrentPosition();
        double eror = dsVal - mvVal;*/

        /*
        if (ea != 0){
            dsVal = ArmMotor.getCurrentPosition();
        }
        if (gamepad2.a){
            ArmMotor.setPower(0);
            dsVal = ArmMotor.getCurrentPosition();
        }
        if (!gamepad2.a) {
            if (ea == 0) {
                double op = 0.0005;
                double od = 0.0005;

                double errod = eror - prevEr;
                prevEr = eror;
                double errop = eror;

                double Go = (op * errop) + (od * errod);
                ArmMotor.setPower(Go);
                mvVal = ArmMotor.getCurrentPosition();
                telemetry.addData("Arm Value", ArmMotor.getCurrentPosition());
            }
        }*/
    }
    public void Extend(){
        double outVal = -1310;
        double inVal = 0;
        double lenVal = ExtendArm.getCurrentPosition();

        if(gamepad2.dpad_up){
            //wanVal = outVal;
            if (ExtendArm.getCurrentPosition() > -1310){
                ExtendArm.setPower(-1);
            }
            else{
                ExtendArm.setPower(0);
            }
        }
        else if(gamepad2.dpad_down){
            //wanVal = inVal;
            if(ExtendArm.getCurrentPosition() < 0){
                ExtendArm.setPower(1);
            }
            else{
                ExtendArm.setPower(0);
            }
        }
        else {
            ExtendArm.setPower(0);
        }
        telemetry.addData("Extend Value", ExtendArm.getCurrentPosition());
        /*double extend = wanVal - lenVal;

        double p = 0.01;
        double d = 0.01;

        double extD = extend - prevExt;
        prevExt = extend;
        double extP = extend;

        double move = (p * extP) + (d * extD);
        ExtendArm.setPower(move);
        lenVal = ExtendArm.getCurrentPosition();
        telemetry.addData("Extend Value", lenVal);*/
    }

    public void Grabber() {
        if(gamepad2.right_bumper) {
            grabber.setPower(-1);
        }
        else if(gamepad2.left_bumper){
            grabber.setPower(1);
        }
        else{
            grabber.setPower(0);
        }
    }
}
