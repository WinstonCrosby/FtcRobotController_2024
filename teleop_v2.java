package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TeleOp")
public class teleop_v2 extends OpMode {
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

    //Define extension limits
    double limitLow = -100;
    double limitMid = -1300;
    double limitHigh = -100;

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

    //initialize
    @Override
    public void init() {
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
        
        //Increase or decrease height switch
        if (gamepad2.y && armChange && Height < 4)
        {
            Height += 1;
            armChange = false;
        }
        else if(gamepad2.a && armChange && Height > 0)
        {
            Height -= 1;
            armChange = false;
        }
        else if(!gamepad2.a && !gamepad2.y)
        {
            armChange = true;
        }

        //Report height
        telemetry.addData("Arm Height Mode", Height);

        //Slow down wheel speed if arm is raised
        if (ArmMotor.getCurrentPosition() > 1500)
        {
            x = gamepad1.left_stick_x/4;
            y = gamepad1.left_stick_y/4;
            t = gamepad1.right_stick_x/4;
        }
        else 
        {
            x = gamepad1.left_stick_x / 2;
            y = gamepad1.left_stick_y / 2;
            t = gamepad1.right_stick_x / 2;
        }

        //Check current arm position
        currentArmPosition = ArmMotor.getCurrentPosition();

        //If reset button is pressed reset encoder value
        if (ResetExtend.isPressed())
        {
            ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            PrevButton = 1;
        }
        else 
        {
            ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PrevButton = 0;
        }
        
        telemetry.addData("Sensor", ResetExtend.isPressed());

        //--------------------------------------------------------------------//
        //-------------------------End Input Block----------------------------//
        //--------------------------------------------------------------------//

        //Function used to move the robot
        Drive(x, y, t);

        // Switch Arm Height (maybe)
        switch (Height)
        {
            case 0:
            {
                desArmPosition = armLow;
                telemetry.addData("Arm Value", "Arm is Down");

                Extend(limitLow);
                break;
            }
            case 1:
            {
                desArmPosition = armGrab;
                telemetry.addData("Arm Value", "Arm is Grabbing");

                Extend(limitLow);
                break;
            }
            case 2:
            {
                desArmPosition = armMed;
                telemetry.addData("Arm Value", "Arm is at Low Basket");

                Extend(limitLow);
                break;
            }
            case 3:
            {
                desArmPosition = armHigh;
                telemetry.addData("Arm Value", "Arm is at High Basket");

                Extend(limitMid);
                break;
            }
            case 4:
            {
                desArmPosition = armHang;
                telemetry.addData("Arm Value", "Arm is in Hang Position");

                Extend(limitHigh);
                break;
            }
        }
        //Function used to control the angle of the arm
        //Returns the previous error so that the variable is not overwritten every iteration
        prevArmError = Arm(desArmPosition, currentArmPosition, prevArmError);

        Grabber();

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
    }
    
    public void Extend(double extendLimit)
    {
        double inVal = 0;
        double lenVal = ExtendArm.getCurrentPosition();

        //Extend the arm while the up button is pressed, retract while down button is pressed
        if(gamepad2.dpad_up)
        {
            if (lenVal > extendLimit)
            {
                ExtendArm.setPower(-1);
            }
            else{
                ExtendArm.setPower(0);
            }
        }
        else if(gamepad2.dpad_down)
        {
            if(lenVal < inVal)
            {
                ExtendArm.setPower(1);
            }
            else
            {
                ExtendArm.setPower(0);
            }
        }
        else 
        {
            ExtendArm.setPower(0);
        }
        telemetry.addData("Extend Value", lenVal);
    }

    public void Grabber() 
    {
        //Intake with right bumper, output with left bumper
        if(gamepad2.right_bumper) 
        {
            grabber.setPower(-1);
        }
        else if(gamepad2.left_bumper)
        {
            grabber.setPower(1);
        }
        else
        {
            grabber.setPower(0);
        }
    }
}
