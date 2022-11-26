package org.firstinspires.ftc.teamcode.buttonSwitch;

public abstract class FirstButtonSwitch {
        public boolean lastButtonState;
        private boolean activate = false;

        public boolean getState(boolean buttonState) {
            if (buttonState && !lastButtonState) { activate = !activate; }
            lastButtonState = buttonState;
            return activate;
        }
}
