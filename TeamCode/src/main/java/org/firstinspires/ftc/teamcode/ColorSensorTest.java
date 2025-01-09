package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ColorSensorTest", group = "TeleOp")

public class ColorSensorTest extends LinearOpMode {
    private Servo Claw;
    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; // Light Intensity
    private double targetValue = 200;

    @Override
    public void runOpMode(){
        initHardware();
        while (!isStarted()) {
            getColor();
            colorTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            getColor();
            colorTelemetry();
            teleOpControls();
        }
    }

    public void initHardware( ){
        initColorSensor();
        initClawServo();
    }

    public void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ClawColor");
    }

    public void initClawServo() {
        Claw = hardwareMap.get(Servo.class, "Claw");
    }

    public void getColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }

    public void colorTelemetry() {
        telemetry.addData("redValue","%.2f", redValue);
        telemetry.addData("greenValue","%.2f", greenValue);
        telemetry.addData("blueValue","%.2f", blueValue);
        telemetry.addData("alphaValue","%.2f", alphaValue);
        telemetry.update();

    }

    public void teleOpControls() {
        if(alphaValue> targetValue) {
            Claw.setPosition(.5);
        }
        else{
            Claw.setPosition(.2);
        }
    }

}
