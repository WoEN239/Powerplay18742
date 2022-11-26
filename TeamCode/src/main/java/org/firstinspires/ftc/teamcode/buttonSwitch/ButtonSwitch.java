package org.firstinspires.ftc.teamcode.buttonSwitch;

import android.os.Build;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class ButtonSwitch extends FirstButtonSwitch {
        BooleanSupplier supplier;
        Consumer<Boolean> consumer;

        public ButtonSwitch(BooleanSupplier supp, Consumer<Boolean> cons) {
            this.supplier = supp;
            this.consumer = cons;
        }

        public void activate() {
            boolean bool = false;
            bool = supplier.getAsBoolean();
            consumer.accept(super.getState(bool));
        }
}
