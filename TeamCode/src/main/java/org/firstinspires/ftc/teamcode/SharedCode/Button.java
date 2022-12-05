package org.firstinspires.ftc.teamcode.SharedCode;

public class Button {
    // for abstracting button management
    // TODO: find a way to store a reference to a bool so the button does not have to be in every function parameter
    Button() {}

    boolean wasPressed = false;
    boolean toggle = false;

    boolean OnButtonDown(boolean button)
    {
        boolean ret = false;
        if (!wasPressed)
        {
            ret = true;
            toggle = !toggle;
        }
        wasPressed = button;

        return ret;
    }

    boolean OnButtonUp(boolean button)
    {
        boolean ret = false;
        if (wasPressed)
        {
            ret = !button;
        }
        wasPressed = button;

        return ret;
    }

    boolean OnToggleOn(boolean button)
    {
        return toggle;
    }

    boolean OnToggleOff(boolean button)
    {
        return !toggle;
    }

}
