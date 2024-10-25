package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@TeleOp(name = "TeleOp")
public class test_thingy extends OpMode {
    //name motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor ArmMotor;
    DcMotor ExtendArm;
    CRServo grabber;
    Servo wrist;
    //name servos


    double desVal = 0;
    double wanVal = 0;

    double prevErr = 0;
    double prevExt = 0;
    //initialize
    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        wrist = hardwareMap.get(Servo.class, "wrist");

        telemetry.addData("Hardware", "much gud");
    }

        //loop start
        @Override
        public void loop () {

            float y = gamepad1.left_stick_y;
            float x = gamepad1.left_stick_x;
            float t = gamepad1.right_stick_x;
            float ea = gamepad2.right_stick_y;
            ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //up and down, left right, and turning
            motorFL.setPower(y - x - t);
            motorFR.setPower(-y - x - t);
            motorBL.setPower(y + x - t);
            motorBR.setPower(-y + x - t);
            ArmMotor.setPower(ea);


            //Arm up define arm values
            double upVal = 1526;
            double dwnVal = 0;
            double outVal = 400;
            double inVal = 0;
            double enVal = ArmMotor.getCurrentPosition();
            double lenVal = ExtendArm.getCurrentPosition();

            // If Y button is pressed, increase arm motor position
            if(gamepad2.triangle) {
                desVal = upVal;
            }
            // If A button is pressed, increase arm motor position
            if(gamepad2.cross) {
                desVal = dwnVal;
            }

            // If X button held, make grabber pickup
            if(gamepad2.square){
                grabber.setPower(-2);
            }

            //If B button held, make grabber spit out
            else if(gamepad2.circle){
                grabber.setPower(+2);
            }

            else{
                grabber.setPower(0);
            }

            if (gamepad2.dpad_left){
                wrist.setPosition(0.43);
            }

            if (gamepad2.dpad_right) {
                wrist.setPosition(0.75);
            }

            if(gamepad2.dpad_up){
                wanVal = outVal;
            }

            if(gamepad2.dpad_down){
                wanVal = inVal;
            }

            double extend = wanVal - lenVal;
            double error = desVal - enVal;

            //Arm up/down stuff
            double P = 0.005;
            double D = 0.005;

            double errD = error - prevErr;
            prevErr = error;
            double errP = error;




            double Imp = (P * errP) + (D * errD);
            ArmMotor.setPower(Imp);
            enVal = ArmMotor.getCurrentPosition();
            telemetry.addData("Arm Value", enVal);

            //Arm in/out stuff
            double p = 0.005;
            double d = 0.005;

            double extD = extend - prevExt;
            prevExt = extend;
            double extP = extend;

            double move = (p * extP) + (d * extD);
            ExtendArm.setPower(move);
            lenVal = ExtendArm.getCurrentPosition();
            telemetry.addData("Extend Value", lenVal);

        }
        }