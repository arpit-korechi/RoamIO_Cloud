package com.korechi.roamiocloud;

import androidx.appcompat.app.AppCompatActivity;

import android.hardware.display.VirtualDisplay;
import android.os.Bundle;
import android.widget.TextView;

import com.korechi.roamiocloud.views.VirtualJoystickView;

public class MainActivity extends AppCompatActivity {

    private VirtualJoystickView joystickView; //Rosjava implementation of a Virtual Joystick
    TextView textX, textY, textZ;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textX = findViewById(R.id.textX);
        textY = findViewById(R.id.textY);
        textZ = findViewById(R.id.textZ);

        joystickView = findViewById(R.id.left_virtual_joystick);
        if (joystickView.currentVelocityCommand != null) {
            double angularX = joystickView.currentVelocityCommand.getAngular().getX();
            double angularY = joystickView.currentVelocityCommand.getAngular().getY();
            double angularZ = joystickView.currentVelocityCommand.getAngular().getZ();

            double linearX = joystickView.currentVelocityCommand.getLinear().getX();
            double linearY = joystickView.currentVelocityCommand.getLinear().getY();
            double linearZ = joystickView.currentVelocityCommand.getLinear().getZ();

            textX.setText("X : " + angularX);
            textY.setText("X : " + angularY);
            textZ.setText("X : " + angularZ);
        }

    }
}