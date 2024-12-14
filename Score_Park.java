package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous
public class Score_Park extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor ArmMotor;
    CRServo grabber;

    double up = 1930;
    double preErr = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber = hardwareMap.get(CRServo.class, "grabber");

    }

    @Override
    public void start() {

        timer.reset();
        while (timer.seconds() < 1.5) {

            motorFL.setPower(-0.30);
            motorFR.setPower(0.30);
            motorBL.setPower(-0.30);
            motorBR.setPower(0.30);

        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        timer.reset();
        while (timer.seconds() < 0.5) {
            double setVal = ArmMotor.getCurrentPosition();
            double ep = up - setVal;

            double p = 0.0005;
            double d = 0.0005;

            double ed = ep - preErr;
            preErr = ep;

            double UP = (p * ep) + (d * ed);
            ArmMotor.setPower(UP);
        }
        ArmMotor.setPower(0);

        timer.reset();
        while (timer.seconds() < 2) {
            grabber.setPower(0.75);
        }
        grabber.setPower(0);

        timer.reset();
        while (timer.seconds() < 3) {

            motorFL.setPower(0.40);
            motorFR.setPower(-0.40);
            motorBL.setPower(0.40);
            motorBR.setPower(-0.40);

        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }

    public void loop() {}

}
