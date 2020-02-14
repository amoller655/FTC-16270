package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Selector {
    private Gamepad gamepad;
    private Telemetry telemetry;

    private String[] Selections = {"Red", "Loading Zone", "0.0"};
    private String[] AllianceOptions = {"Red", "Blue"};
    private String AllianceSelection = "Red";
    private int AllianceIndex = 0;
    private String[] StartingZoneOptions = {"Loading Zone", "Building Zone"};
    private String StartingZoneSelection = "Loading Zone";
    private int StartingZoneIndex = 0;
    private double InitialDelay = 0.0;

    private String[] Categories = {"  Alliance: ", "  Starting Zone: ", "  Initial Delay: "};
    private int Category = 0;
    private int CategoryMin = 0;
    private int CategoryMax = Categories.length - 1;

    private boolean lock = false;
    private boolean pressed = false;

    public Selector(Gamepad gamepad, Telemetry telemetry)
    {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }
    public void selectionLoop()
    {
        if(!lock)
        {
            checkInputs();
            updateCategory();
            updateTelemetry();
        }
    }
    private void checkInputs()
    {

        if(gamepad.left_stick_y > .5)
            changeCategory(+1);
        else if(gamepad.left_stick_y < -.5)
            changeCategory(-1);
        else if(gamepad.left_stick_x < -.5)
            changeOption(-1);
        else if(gamepad.left_stick_x > .5)
            changeOption(+1);
        else if(gamepad.start)
            lockSelection();

    }
    private void changeCategory(int direction)
    {
        if(direction < 0)
        {
            Category -= 1;
            if(Category < CategoryMin)
            {
                Category = CategoryMax;
            }
        }
        else if(direction > 0)
        {
            Category += 1;
            if(Category > CategoryMax)
            {
                Category = CategoryMin;
            }
        }
    }
    private void updateCategory()
    {
        for(int i = CategoryMin; i <= CategoryMax; i++)
        {
            if(i == Category && !lock)
            {
                Categories[i] = "<>" + Categories[i].substring(2);
            }
            else
            {
                Categories[i] = "  " + Categories[i].substring(2);
            }
        }
    }
    private void changeOption(int direction)
    {
        if(direction < 0)
        {
            switch(Category)
            {
                case 0:
//                    Alliance
                    AllianceIndex -= 1;
                    if(AllianceIndex < 0)
                    {
                        AllianceIndex = AllianceOptions.length - 1;
                    }
                    AllianceSelection = AllianceOptions[AllianceIndex];
                    Selections[0] = AllianceSelection;
                    break;
                case 1:
//                    Starting Zone
                    StartingZoneIndex -= 1;
                    if(StartingZoneIndex < 0)
                    {
                        StartingZoneIndex = StartingZoneOptions.length - 1;
                    }
                    StartingZoneSelection = StartingZoneOptions[StartingZoneIndex];
                    Selections[1] = StartingZoneSelection;
                    break;
                case 2:
//                    Initial Delay
                    InitialDelay -= .5;
                    if(InitialDelay < 0)
                    {
                        InitialDelay = 30.0;
                    }
                    Selections[2] = Double.toString(InitialDelay);
                    break;
            }
        }
        else if(direction > 0)
        {
            switch(Category)
            {
                case 0:
//                    Alliance
                    AllianceIndex += 1;
                    if(AllianceIndex > AllianceOptions.length - 1)
                    {
                        AllianceIndex = 0;
                    }
                    AllianceSelection = AllianceOptions[AllianceIndex];
                    Selections[0] = AllianceSelection;
                    break;
                case 1:
//                    Starting Zone
                    StartingZoneIndex += 1;
                    if(StartingZoneIndex > StartingZoneOptions.length - 1)
                    {
                        StartingZoneIndex = 0;
                    }
                    StartingZoneSelection = StartingZoneOptions[StartingZoneIndex];
                    Selections[1] = StartingZoneSelection;
                    break;
                case 2:
//                    Initial Delay
                    InitialDelay += .5;
                    if(InitialDelay > 30.0)
                    {
                        InitialDelay = 0.0;
                    }
                    Selections[2] = Double.toString(InitialDelay);
                    break;
            }
        }
    }

    private void lockSelection()
    {
        lock = true;
    }
    private void updateTelemetry()
    {
        for(int i = CategoryMin; i <= CategoryMax; i++)
        {
            telemetry.addData(Categories[i], Selections[i]);
        }
        if(lock)
        {
            telemetry.addLine("Selection locked in. Ready to run!");
        }
        telemetry.update();
    }
}
